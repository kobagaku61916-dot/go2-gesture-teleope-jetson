"""YOLO-pose TensorRT のカメラ込みベンチマーク（Jetson 用・Go2 非依存）.

使い方（venv で。ROS 不要）:
    venv/bin/python3 scripts/bench_yolo.py --engine yolo11n-pose.fp16.engine --device 4
検出結果と各段の処理時間・実効 fps を表示する。
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402

from src.detect.yolo_pose_tracker import YoloPoseEngine  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engine", default="yolo11n-pose.fp16.engine")
    ap.add_argument("--device", type=int, default=4)
    ap.add_argument("--frames", type=int, default=120)
    args = ap.parse_args()

    eng = YoloPoseEngine(args.engine)
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ok, img = cap.read()
    assert ok, "カメラを開けない"

    for _ in range(10):  # ウォームアップ
        ok, img = cap.read()
        eng.infer(img)

    t_cap = t_inf = 0.0
    n_det = 0
    t0 = time.time()
    for _ in range(args.frames):
        t = time.time()
        ok, img = cap.read()
        t_cap += time.time() - t
        t = time.time()
        dets = eng.infer(img)
        t_inf += time.time() - t
        n_det += len(dets)
    wall = time.time() - t0

    print(f"frames={args.frames}  実効 {args.frames / wall:.1f} fps")
    print(f"  カメラ取得: {t_cap / args.frames * 1000:.1f} ms/フレーム")
    print(f"  推論+前後処理: {t_inf / args.frames * 1000:.1f} ms/フレーム")
    print(f"  平均検出人数: {n_det / args.frames:.2f}")
    if dets:
        bbox, kpts, conf = dets[0]
        ls, rs = kpts[5], kpts[6]
        print(f"  例: conf={conf:.2f} bbox={[int(v) for v in bbox]} "
              f"肩幅={abs(ls[0] - rs[0]):.0f}px")
    eng.close()
    cap.release()


if __name__ == "__main__":
    main()
