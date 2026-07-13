"""YOLO-pose (TensorRT) 推論ラッパ — PoseTracker 互換インターフェース.

Jetson に torch を入れないための構成（requirements-jetson.md の方針を踏襲）:
    Desktop: ultralytics で ONNX エクスポート（scripts/export_yolo.md 参照）
    Jetson : trtexec で FP16 エンジン化 → 本モジュールが TensorRT 10 + cuda.bindings で推論

出力は yolo11n-pose / yolov8n-pose 共通の (1, 56, 8400):
    56 = cx, cy, w, h, conf, 17 キーポイント × (x, y, conf)

クラス構成:
- YoloPoseEngine: 前処理(レターボックス) → 推論 → 後処理(信頼度フィルタ+NMS)。
  検出リスト [(bbox, kpts17, conf)]（元画像ピクセル座標）を返す
- YoloPoseTracker: PoseTracker と同じ find_pose / find_position /
  find_visibilities / close を提供。内部で PersonTracker が対象 1 人を選び、
  coco_adapter で MediaPipe 形式に変換する → 下流（follow/gesture）無改造
"""

import time

import cv2
import numpy as np
import tensorrt as trt
from cuda.bindings import runtime as cudart

from .coco_adapter import coco_to_lm_list
from .person_tracker import PersonTracker, TrackerParams

INPUT_SIZE = 640


def _check(err):
    code = err[0] if isinstance(err, tuple) else err
    if int(code) != 0:
        raise RuntimeError(f"CUDA error: {code}")
    return err[1] if isinstance(err, tuple) and len(err) > 1 else None


class YoloPoseEngine:
    """TensorRT エンジンの実行（単バッチ・同期）."""

    def __init__(self, engine_path: str, conf_th: float = 0.4, iou_th: float = 0.5):
        self._conf_th = conf_th
        self._iou_th = iou_th
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self._engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        self._ctx = self._engine.create_execution_context()

        self._io = {}
        for i in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(i)
            shape = tuple(self._engine.get_tensor_shape(name))
            nbytes = int(np.prod(shape)) * 4  # fp32 入出力
            dptr = _check(cudart.cudaMalloc(nbytes))
            self._ctx.set_tensor_address(name, int(dptr))
            mode = self._engine.get_tensor_mode(name)
            self._io[name] = (dptr, nbytes, shape, mode)
            if mode == trt.TensorIOMode.INPUT:
                self._in_name, self._in_shape = name, shape
            else:
                self._out_name, self._out_shape = name, shape
        self._stream = _check(cudart.cudaStreamCreate())
        self._out_host = np.empty(self._out_shape, dtype=np.float32)

    def infer(self, img_bgr):
        """1 フレーム推論して検出リストを返す.

        Returns:
            [(bbox(x1,y1,x2,y2), kpts[(x,y,conf)]×17, conf)]（元画像ピクセル座標）
        """
        blob, ratio, pad = self._preprocess(img_bgr)
        d_in, nbytes, _, _ = self._io[self._in_name]
        _check(cudart.cudaMemcpyAsync(
            int(d_in), blob.ctypes.data, nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self._stream))
        self._ctx.execute_async_v3(int(self._stream))
        d_out, out_nbytes, _, _ = self._io[self._out_name]
        _check(cudart.cudaMemcpyAsync(
            self._out_host.ctypes.data, int(d_out), out_nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self._stream))
        _check(cudart.cudaStreamSynchronize(int(self._stream)))
        return self._postprocess(self._out_host, ratio, pad)

    def _preprocess(self, img):
        h, w = img.shape[:2]
        r = min(INPUT_SIZE / w, INPUT_SIZE / h)
        nw, nh = int(round(w * r)), int(round(h * r))
        pad_x, pad_y = (INPUT_SIZE - nw) / 2, (INPUT_SIZE - nh) / 2
        resized = cv2.resize(img, (nw, nh))
        canvas = np.full((INPUT_SIZE, INPUT_SIZE, 3), 114, dtype=np.uint8)
        top, left = int(round(pad_y - 0.1)), int(round(pad_x - 0.1))
        canvas[top:top + nh, left:left + nw] = resized
        blob = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = np.ascontiguousarray(blob.transpose(2, 0, 1)[None])  # (1,3,640,640)
        return blob, r, (left, top)

    def _postprocess(self, out, ratio, pad):
        pred = out[0].T                     # (8400, 56)
        mask = pred[:, 4] >= self._conf_th
        pred = pred[mask]
        if pred.shape[0] == 0:
            return []
        # xywh → xyxy（レターボックス座標）
        cx, cy, w, h = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
        boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)
        keep = self._nms(boxes, pred[:, 4])
        results = []
        for i in keep:
            x1, y1, x2, y2 = boxes[i]
            bbox = ((x1 - pad[0]) / ratio, (y1 - pad[1]) / ratio,
                    (x2 - pad[0]) / ratio, (y2 - pad[1]) / ratio)
            kp = pred[i, 5:5 + 51].reshape(17, 3)
            kpts = [((float(x) - pad[0]) / ratio, (float(y) - pad[1]) / ratio, float(c))
                    for x, y, c in kp]
            results.append((bbox, kpts, float(pred[i, 4])))
        return results

    def _nms(self, boxes, scores):
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(int(i))
            if order.size == 1:
                break
            xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
            yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
            xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
            yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
            area_o = ((boxes[order[1:], 2] - boxes[order[1:], 0]) *
                      (boxes[order[1:], 3] - boxes[order[1:], 1]))
            iou = inter / np.maximum(area_i + area_o - inter, 1e-9)
            order = order[1:][iou <= self._iou_th]
        return keep

    def close(self):
        for dptr, *_ in self._io.values():
            cudart.cudaFree(int(dptr))
        cudart.cudaStreamDestroy(int(self._stream))


class YoloPoseTracker:
    """PoseTracker 互換の facade（対象選択 + MediaPipe 形式変換込み）."""

    def __init__(self, engine_path: str, conf_th: float = 0.4,
                 tracker_params: TrackerParams = TrackerParams()):
        self._engine = YoloPoseEngine(engine_path, conf_th=conf_th)
        self._tracker = PersonTracker(tracker_params)
        self._lm = []
        self._vis = []

    def find_pose(self, img, draw: bool = False):
        detections = self._engine.infer(img)
        target = self._tracker.update(
            [(d[0], d) for d in detections], time.monotonic())
        if target is None:
            self._lm, self._vis = [], []
        else:
            _, (bbox, kpts, conf) = target
            self._lm, self._vis = coco_to_lm_list(kpts)
            if draw:
                x1, y1, x2, y2 = (int(v) for v in bbox)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 220, 255), 2)
                for x, y, c in kpts:
                    if c > 0.3:
                        cv2.circle(img, (int(x), int(y)), 3, (0, 255, 0), -1)
        return img

    def find_position(self, img, draw: bool = False):
        return self._lm

    def find_visibilities(self):
        return self._vis

    def release_lock(self):
        """対象ロックを解放する（探索モード用）.

        探索旋回中は「次に見えた人を即再捕捉」したいが、IoU 連続性による
        ロック維持（乗り移り防止）が再発見を最大 1 秒弾いてしまうため、
        探索状態の間は呼び出し側がこれを毎フレーム呼んでロックを外す。
        """
        self._tracker.reset()

    def close(self):
        self._engine.close()
