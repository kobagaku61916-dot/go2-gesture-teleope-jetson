"""go2_bridge ノード（プロセス3）— Go2 SportMode API への変換器.

旧 Jetson 資産 sport_mode_bridge_node.py（Move 1008）と action_bridge.py
（Dance 等。旧リポ jetson/ の参照コピー）を 1 プロセスに統合した移植。
どちらも enP8p1s0 参加者だったため統合できる（scripts/run_go2_bridge.sh）。

⚠️ このノード自体に安全機構はない（クランプ・watchdog・許可ゲートは safety_gate 側）。
   必ず safety_gate を経由した /cmd_vel_robot・/go2_action_robot だけを購読する。
   **起動は人間のみ**（runbook.md §0）。

api_id は Jetson 上の unitree_ros2 ros2_sport_client.h / go2_robot_sdk
robot_commands.py の 2 ソースで実測確認済み（2026-07-06）。Dance/Hello 等は
parameter なし（api_id のみ）で実行する SDK 実装と同形。
"""

import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from unitree_api.msg import Request

from src.config import load_section


class Go2Bridge(Node):
    def __init__(self, cfg: dict):
        super().__init__("go2_bridge")
        self._move_api_id = int(cfg["move_api_id"])
        self._action_api_ids = {str(k): int(v)
                                for k, v in cfg["action_api_ids"].items()}
        self._pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.create_subscription(Twist, "/cmd_vel_robot", self._on_cmd, 10)
        self.create_subscription(String, "/go2_action_robot", self._on_action, 10)
        self.get_logger().info(
            f"go2_bridge: Move={self._move_api_id}, "
            f"actions={sorted(self._action_api_ids)}")

    def _on_cmd(self, msg: Twist):
        req = Request()
        req.header.identity.api_id = self._move_api_id
        req.parameter = json.dumps({
            "x": float(msg.linear.x),
            "y": float(msg.linear.y),
            "z": float(msg.angular.z),
        })
        self._pub.publish(req)

    def _on_action(self, msg: String):
        name = msg.data.strip().lower()
        api_id = self._action_api_ids.get(name)
        if api_id is None:
            # 二重許可リスト（safety_gate が正しくても bridge 側でも破棄する）
            self.get_logger().warn(f"拒否(対応表にない): {name!r}")
            return
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = ""
        self._pub.publish(req)
        self.get_logger().info(f"実行: {name} (api_id={api_id})")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Go2Bridge(load_section("go2_bridge"))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
