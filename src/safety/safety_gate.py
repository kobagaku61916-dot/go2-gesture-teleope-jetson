"""safety_gate ノード（プロセス2）— 速度クランプ + watchdog + アクションゲート.

旧 Jetson 資産 cmd_vel_relay.py（クランプ 0.6/0.8・watchdog 0.5s・終了時 0 送出）と
action_relay.py（許可リスト・cooldown・静止確認）を 1 プロセスに統合した移植。
DDS は lo 単独参加者で動かす（scripts/run_safety_gate.sh）。

本リポでの追加（旧 T1 の教訓）:
- SIGHUP / SIGTERM / SIGINT のいずれでも「0 を送出してから」終了する
  （旧実装は KeyboardInterrupt 経路のみで、SSH 切断の SIGHUP では 0 が出なかった）。
"""

import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from src.config import load_section


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SafetyGate(Node):
    def __init__(self, cfg: dict):
        super().__init__("safety_gate")
        self._max_linear = float(cfg["max_linear"])
        self._max_angular = float(cfg["max_angular"])
        self._watchdog_timeout = float(cfg["watchdog_timeout_sec"])
        self._allowlist = tuple(cfg["action_allowlist"])
        self._cooldown = float(cfg["action_cooldown_sec"])
        self._stop_hold = float(cfg["action_stop_hold_sec"])
        self._bypass = tuple(cfg["action_bypass"])

        # --- 速度経路（クランプ + watchdog）---
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_robot", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self._last_rx = self.get_clock().now()
        self._stopped = True
        self.create_timer(0.1, self._watchdog)

        # --- アクション経路（許可リスト + cooldown + 静止確認）---
        self._act_pub = self.create_publisher(String, "/go2_action_robot", 10)
        self.create_subscription(String, "/go2_action", self._on_action, 10)
        self._last_pass = None
        self._last_nonzero = 0.0

        self.get_logger().info(
            f"safety_gate: clamp {self._max_linear}/{self._max_angular}, "
            f"watchdog {self._watchdog_timeout}s, allow={self._allowlist}")

    # --- 速度経路 -----------------------------------------------------
    def _on_cmd(self, msg: Twist):
        out = Twist()
        out.linear.x = clamp(float(msg.linear.x), -self._max_linear, self._max_linear)
        out.linear.y = clamp(float(msg.linear.y), -self._max_linear, self._max_linear)
        out.angular.z = clamp(float(msg.angular.z), -self._max_angular, self._max_angular)
        self._cmd_pub.publish(out)
        self._last_rx = self.get_clock().now()
        self._stopped = False
        if any((out.linear.x, out.linear.y, out.angular.z)):
            self._last_nonzero = time.monotonic()

    def _watchdog(self):
        elapsed = (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9
        if elapsed > self._watchdog_timeout and not self._stopped:
            self._cmd_pub.publish(Twist())  # 指令途絶 → 停止を 1 回送る
            self._stopped = True
            self.get_logger().warn("watchdog: /cmd_vel 途絶 -> 停止を送信")

    # --- アクション経路 ------------------------------------------------
    def _on_action(self, msg: String):
        name = msg.data.strip().lower()
        now = time.monotonic()
        if name not in self._allowlist:
            self.get_logger().warn(f"拒否(未許可): {name!r}")
            return
        if name not in self._bypass:
            if self._last_pass is not None and now - self._last_pass < self._cooldown:
                self.get_logger().warn(
                    f"拒否(cooldown 残り {self._cooldown - (now - self._last_pass):.1f}s): {name}")
                return
            if now - self._last_nonzero < self._stop_hold:
                self.get_logger().warn(
                    f"拒否(移動指令が直近 {self._stop_hold}s 内): {name}")
                return
            self._last_pass = now
        out = String()
        out.data = name
        self._act_pub.publish(out)
        self.get_logger().info(f"通過: {name}")

    # --- 終了時の安全挙動 ----------------------------------------------
    def send_stop(self):
        if rclpy.ok():
            self._cmd_pub.publish(Twist())


def main(args=None):
    # SIGHUP でも 0 送出してから終了する（SSH 切断対策 = 旧 T1）
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def _raise_interrupt(_signum, _frame):
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _raise_interrupt)

    node = None
    try:
        node = SafetyGate(load_section("safety_gate"))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.send_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
