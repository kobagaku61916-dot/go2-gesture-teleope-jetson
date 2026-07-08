#!/usr/bin/env bash
# [Jetson] go2_bridge（Go2 SportMode API 変換）。DDS: enP8p1s0 単独 + Peer 127.0.0.1
#   （旧リポ T2 の実証構成。lo を束ねると Go2 を発見できない）。
# ※ このスクリプトを起動するとロボットが動きうる。
#   周囲の安全・リモコン保持を確認のうえ **人間が** tmux 内で実行すること（runbook.md §0）。
set -e
cd "$(dirname "$0")/.."
mkdir -p logs
source /opt/ros/humble/setup.bash
source ~/go2_guide_dog/unitree_ros2/setup.sh >/dev/null 2>&1   # unitree_api msgs
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP8p1s0"/></Interfaces></General><Discovery><Peers><Peer address="127.0.0.1"/></Peers><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>'
ros2 daemon stop >/dev/null 2>&1 || true
echo "[go2_bridge] /cmd_vel_robot, /go2_action_robot -> /api/sport/request  起動（実機が動きうる）"
exec venv/bin/python3 -m src.bridge.go2_bridge 2>&1 | tee -a "logs/go2_bridge_$(date +%Y%m%d).log"
