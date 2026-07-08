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
from src.gesture.dance_detector import DanceDetector, DanceParams
from src.gesture.debounce import CommandDebouncer
from src.gesture.gesture_mapper import GestureParams, compute_command
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

        self._camera = create_camera(
            camera_type=str(cam["type"]), device=int(cam["device"]),
            width=int(cam["width"]), height=int(cam["height"]),
            fourcc=str(cam.get("fourcc", "")))
        if not self._camera.open():
            raise RuntimeError(
                f"カメラを開けませんでした: {self._camera!r}。"
                "configs の camera.device を確認（YUYV ノード。runbook Phase 1）")
        self.get_logger().info(f"camera: {self._camera!r}")

        self._tracker = PoseTracker(
            model_complexity=int(pose["model_complexity"]),
            detection_confidence=float(pose["detection_confidence"]),
            tracking_confidence=float(pose["tracking_confidence"]))

        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- アクション（ダンス）---
        act = cfg.get("action", {})
        self._action_pub = None
        self._dance_detector = None
        if bool(act.get("enable", False)):
            dc = cfg["dance"]
            self._dance_action = str(act.get("dance", "dance1"))
            self._action_pub = self.create_publisher(String, "/go2_action", 10)
            self._dance_detector = DanceDetector(DanceParams(
                extend_ratio=float(dc["extend_ratio"]),
                retract_ratio=float(dc["retract_ratio"]),
                min_duration_sec=float(dc["min_duration_sec"]),
                min_swaps=int(dc["min_swaps"]),
                max_interval_sec=float(dc["max_interval_sec"]),
                cooldown_sec=float(dc["cooldown_sec"])))
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

        # 信頼度チェック: 主要ランドマークが低信頼なら NO BODY 扱い
        if lm_list and not key_landmarks_visible(
                self._tracker.find_visibilities(), self._min_visibility):
            lm_list = []

        vx, vy, omega, label = compute_command(lm_list, w, h, self._gesture_params)

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
            import cv2
            color = (0, 255, 0) if label not in ("STOP", "NO BODY") else (0, 0, 255)
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
    parser.add_argument("--no-low-speed", action="store_true",
                        help="低速モード解除（configs の速度を使う）")
    args = parser.parse_args()

    cfg = load_section("gesture_node", args.config)
    if args.device is not None:
        cfg["camera"]["device"] = args.device
    if args.display:
        cfg["display"] = True
    if args.enable_action:
        cfg.setdefault("action", {})["enable"] = True
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
