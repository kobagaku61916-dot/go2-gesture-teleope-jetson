"""gesture_node（プロセス1）— 認識ノード本体.

camera(V4L2) → PoseTracker(MediaPipe 0.10.18) → 信頼度チェック
→ gesture_mapper（速度）+ debounce / dance_detector（アクション）
→ /cmd_vel (Twist) ・ /go2_action (String) を publish。

旧リポ gesture_teleop_node.py の骨格を移植し、本リポの安全追加
（信頼度チェック・debounce・低速モード・SIGHUP 対応）を組み込んだ版。
ヘッドレス既定（display は開発時のみ）。設定は configs/params.yaml。

安全挙動:
- ノード終了（SIGINT/SIGTERM/SIGHUP いずれも）時に必ず 0 を publish してから落ちる
- フレーム取得失敗時は 0 を publish
- 姿勢の主要ランドマーク visibility が下限未満 → NO BODY 扱い（0）
- 非ゼロ指令は debounce_frames 連続一致ではじめて送出（STOP は即時）
- low_speed_mode 中は 0.2/0.3 に強制
"""

import argparse
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from src.camera import create_camera
from src.config import load_section
from src.follow.follow_controller import FollowController, FollowParams
from src.gesture.dance_detector import DanceDetector, DanceParams
from src.gesture.debounce import CommandDebouncer
from src.gesture.gesture_mapper import GestureParams, compute_command
from src.gesture.wave_detector import WaveDetector, WaveParams
from src.pose.confidence import key_landmarks_visible
from src.pose.pose_tracker import PoseTracker

LOW_SPEED = (0.2, 0.3)   # low_speed_mode 時の (linear, angular)
WINDOW_NAME = "gesture_node (q: quit)"


class GestureNode(Node):
    def __init__(self, cfg: dict):
        super().__init__("gesture_node")
        cam = cfg["camera"]
        pose = cfg["pose"]

        if bool(cfg.get("low_speed_mode", True)):
            linear, angular = LOW_SPEED
            self.get_logger().warn(
                f"低速モード: linear={linear} angular={angular} に強制"
                "（解除は configs の low_speed_mode: false）")
        else:
            linear = float(cfg["linear_speed"])
            angular = float(cfg["angular_speed"])

        self._gesture_params = GestureParams(
            linear_speed=linear, angular_speed=angular,
            raise_margin=float(cfg["raise_margin"]),
            raise_near=float(cfg["raise_near"]),
            level_margin=float(cfg["level_margin"]),
            extend_margin=float(cfg["extend_margin"]),
        )
        self._min_visibility = float(pose.get("min_visibility", 0.5))
        self._debouncer = CommandDebouncer(int(cfg.get("debounce_frames", 3)))
        self._display = bool(cfg.get("display", False))

        # --- 追従モード（--follow）。テレオペ・アクション検出とは排他 ---
        self._follow = None
        if bool(cfg.get("follow_mode", False)):
            fl = cfg.get("follow", {})
            # 距離校正値は認識バックエンドごとに異なる（肩点の定義が違うため）
            sw_key = ("sw_at_target_yolo"
                      if str(cfg.get("backend", "blazepose")).lower() == "yolo"
                      else "sw_at_target")
            self._follow = FollowController(FollowParams(
                target_distance_m=float(fl.get("target_distance_m", 1.5)),
                sw_at_target=float(fl.get(sw_key, 0.105)),
                deadband_m=float(fl.get("deadband_m", 0.15)),
                center_deadband=float(fl.get("center_deadband", 0.05)),
                k_dist=float(fl.get("k_dist", 0.8)),
                k_yaw=float(fl.get("k_yaw", 2.0)),
                max_vx=float(fl.get("max_vx", 0.6)),
                max_back_vx=float(fl.get("max_back_vx", 0.2)),
                max_omega=float(fl.get("max_omega", 0.8)),
                smooth_alpha=float(fl.get("smooth_alpha", 0.6)),
                lost_grace_sec=float(fl.get("lost_grace_sec", 0.25))))
            self._last_sw_log = 0.0
            self.get_logger().warn(
                f"追従モード v2: 目標 {fl.get('target_distance_m', 1.5)}m "
                f"(sw_at_target={fl.get('sw_at_target', 0.105)})。"
                "テレオペ・アクション検出は無効。見失い猶予 0.25s・探索はしない")
        # 対面操作のミラー: ユーザーから見て手を出した側と同じ方向へ回るよう
        # 旋回符号を反転する（ロボットとユーザーが向かい合う搭載カメラ構成用）
        self._mirror_turns = bool(cfg.get("mirror_turns", True))

        self._camera = create_camera(
            camera_type=str(cam["type"]), device=int(cam["device"]),
            width=int(cam["width"]), height=int(cam["height"]),
            fourcc=str(cam.get("fourcc", "")))
        if not self._camera.open():
            raise RuntimeError(
                f"カメラを開けませんでした: {self._camera!r}。"
                "configs の camera.device を確認（YUYV ノード。runbook Phase 1）")
        self.get_logger().info(f"camera: {self._camera!r}")

        backend = str(cfg.get("backend", "blazepose")).lower()
        if backend == "yolo":
            # TensorRT 依存は選択時のみ import（Desktop でのテスト実行を汚さない）
            from src.config import REPO_ROOT
            from src.detect.yolo_pose_tracker import YoloPoseTracker
            yl = cfg.get("yolo", {})
            engine = str(yl.get("engine", "yolo11n-pose.fp16.engine"))
            self._tracker = YoloPoseTracker(
                str((REPO_ROOT / engine)), conf_th=float(yl.get("conf_th", 0.4)))
            self.get_logger().info(f"認識バックエンド: YOLO-pose TensorRT ({engine})")
        else:
            self._tracker = PoseTracker(
                model_complexity=int(pose["model_complexity"]),
                detection_confidence=float(pose["detection_confidence"]),
                tracking_confidence=float(pose["tracking_confidence"]))
            self.get_logger().info("認識バックエンド: MediaPipe Pose (BlazePose)")

        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- アクション（ダンス・Greet）---
        act = cfg.get("action", {})
        self._action_pub = None
        self._dance_detector = None
        self._greet_detector = None
        if bool(act.get("enable", False)) and self._follow is None:
            wv = cfg.get("wave", {})
            self._greet_action = str(act.get("greet", "hello"))
            self._greet_detector = WaveDetector(WaveParams(
                min_amplitude=float(wv.get("min_amplitude", 0.25)),
                min_swings=int(wv.get("min_swings", 4)),
                min_duration_sec=float(wv.get("min_duration_sec", 2.0)),
                max_gap_sec=float(wv.get("max_gap_sec", 0.8)),
                cooldown_sec=float(wv.get("cooldown_sec", 10.0))))
            dc = cfg["dance"]
            self._dance_action = str(act.get("dance", "dance1"))
            self._action_pub = self.create_publisher(String, "/go2_action", 10)
            self._dance_detector = DanceDetector(DanceParams(
                extend_ratio=float(dc["extend_ratio"]),
                retract_ratio=float(dc["retract_ratio"]),
                min_duration_sec=float(dc["min_duration_sec"]),
                min_swaps=int(dc["min_swaps"]),
                max_interval_sec=float(dc["max_interval_sec"]),
                cooldown_sec=float(dc["cooldown_sec"]),
                no_body_grace_sec=float(dc.get("no_body_grace_sec", 0.7))))
            self.get_logger().info(
                f"dance 検出 有効（左右交互の腕伸ばし）: 発火で '{self._dance_action}'")

        self._last_label = None
        rate = float(cfg.get("rate", 30.0))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"publishing /cmd_vel（実効レートは推論律速 ~13fps）display={self._display}")

    def publish_cmd(self, vx, vy, omega):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(omega)
        self._pub.publish(msg)

    def _tick(self):
        ok, img = self._camera.read()
        if not ok:
            self.get_logger().warn("フレーム取得失敗。停止指令を送信",
                                   throttle_duration_sec=1.0)
            self.publish_cmd(0.0, 0.0, 0.0)
            return

        h, w = img.shape[:2]
        img = self._tracker.find_pose(img, draw=self._display)
        lm_list = self._tracker.find_position(img, draw=False)

        # 信頼度チェック: 主要ランドマークが低信頼なら NO BODY 扱い。
        # 追従モードは両肩(11,12)のみ使うため肩だけ検査（歩行中の手首遮蔽で切れない）
        key_ids = (11, 12) if self._follow is not None else (11, 12, 15, 16)
        if lm_list and not key_landmarks_visible(
                self._tracker.find_visibilities(), self._min_visibility, key_ids):
            lm_list = []

        if self._follow is not None:
            # --- 追従モード: P 制御の連続値をそのまま publish（debounce 不使用）---
            now = time.monotonic()
            fs = self._follow.update(lm_list, w, h, now)
            self.publish_cmd(fs.vx, 0.0, fs.omega)
            if fs.label != self._last_label or now - self._last_sw_log >= 2.0:
                self.get_logger().info(
                    f"follow: {fs.label} dist={fs.distance_m:.2f}m "
                    f"vx={fs.vx:+.2f} omega={fs.omega:+.2f}")
                self._last_label = fs.label
                self._last_sw_log = now
            if self._display:
                self._draw(img, fs.label, h)
            return

        vx, vy, omega, label = compute_command(lm_list, w, h, self._gesture_params)
        if self._mirror_turns:
            omega = -omega
            if "TURN-RIGHT" in label:
                label = label.replace("TURN-RIGHT", "TURN-LEFT")
            elif "TURN-LEFT" in label:
                label = label.replace("TURN-LEFT", "TURN-RIGHT")

        if self._greet_detector is not None:
            gstatus = self._greet_detector.update(lm_list, w, h, time.monotonic())
            if gstatus.waving:
                # 振り中の上げた右手が FORWARD として通らないよう抑制する
                vx, vy, omega = 0.0, 0.0, 0.0
                label = f"WAVE? {int(gstatus.progress * 100)}%"
            if gstatus.triggered:
                msg = String()
                msg.data = self._greet_action
                self._action_pub.publish(msg)
                label = "GREET!"
                self.get_logger().info(
                    f"手振り検出（右手ワイビング）→ action '{self._greet_action}' を送信")

        if self._dance_detector is not None:
            status = self._dance_detector.update(lm_list, w, h, time.monotonic())
            if status.is_active:
                vx, vy, omega = 0.0, 0.0, 0.0
                label = f"DANCE? {int(status.progress * 100)}%"
            if status.triggered:
                msg = String()
                msg.data = self._dance_action
                self._action_pub.publish(msg)
                label = "DANCE!"
                self.get_logger().info(
                    f"ダンス検出 → action '{self._dance_action}' を送信")

        vx, vy, omega = self._debouncer.update((vx, vy, omega))
        self.publish_cmd(vx, vy, omega)

        # ラベルはヘッドレスでも追えるよう、変化時のみログに出す（Phase 3 の確認用）
        if label != self._last_label:
            self.get_logger().info(f"label: {label}")
            self._last_label = label

        if self._display:
            self._draw(img, label, h)

    def _draw(self, img, label, h):
        import cv2
        color = (0, 255, 0) if label not in ("STOP", "NO BODY", "NO TARGET") else (0, 0, 255)
        cv2.putText(img, label, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 3)
        cv2.imshow(WINDOW_NAME, img)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            raise KeyboardInterrupt

    def shutdown(self):
        try:
            if rclpy.ok():
                self.publish_cmd(0.0, 0.0, 0.0)
        finally:
            self._camera.close()
            self._tracker.close()
            if self._display:
                import cv2
                cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="gesture_node（認識ノード）")
    parser.add_argument("--config", default=None, help="params.yaml のパス")
    parser.add_argument("--device", type=int, default=None, help="camera.device 上書き")
    parser.add_argument("--display", action="store_true", help="表示ウィンドウ有効化")
    parser.add_argument("--enable-action", action="store_true", help="ダンス検出有効化")
    parser.add_argument("--follow", action="store_true",
                        help="人追従モード（テレオペ/アクション無効。肩幅ベースの P 制御）")
    parser.add_argument("--backend", choices=["blazepose", "yolo"], default=None,
                        help="認識バックエンド（既定は configs の backend）")
    parser.add_argument("--low-speed", action="store_true",
                        help="低速モード（0.2/0.3 に強制。初回検証・デモ安全用）")
    parser.add_argument("--no-low-speed", action="store_true",
                        help="低速モード解除（configs の速度を使う。既定が最高速のため通常不要）")
    args = parser.parse_args()

    cfg = load_section("gesture_node", args.config)
    if args.device is not None:
        cfg["camera"]["device"] = args.device
    if args.display:
        cfg["display"] = True
    if args.enable_action:
        cfg.setdefault("action", {})["enable"] = True
    if args.follow:
        cfg["follow_mode"] = True
    if args.backend:
        cfg["backend"] = args.backend
    if args.low_speed:
        cfg["low_speed_mode"] = True
    if args.no_low_speed:
        cfg["low_speed_mode"] = False

    # SIGHUP でも 0 送出してから終了（旧 T1 の教訓）
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def _raise_interrupt(_signum, _frame):
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _raise_interrupt)

    node = None
    try:
        node = GestureNode(cfg)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
