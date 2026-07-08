#!/usr/bin/env bash
# [Jetson] safety_gate（クランプ+watchdog+アクションゲート）。
# DDS: wlx + lo（旧リポ relay と同一の実証構成）。
#   lo 単独だと enP8p1s0 参加者（go2_bridge）から発見できない（2026-07-08 実測）。
#   wlx を含めることで実 IP の locator が広告され、bridge と相互発見できる。
# tmux 内で起動すること（runbook.md §2）。
set -e
cd "$(dirname "$0")/.."
mkdir -p logs
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlx6c1ff78a6985" multicast="false"/><NetworkInterface name="lo" multicast="default"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="127.0.0.1"/></Peers><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>'
ros2 daemon stop >/dev/null 2>&1 || true
echo "[safety_gate] /cmd_vel -> clamp+watchdog -> /cmd_vel_robot / action gate  起動"
exec venv/bin/python3 -m src.safety.safety_gate 2>&1 | tee -a "logs/safety_gate_$(date +%Y%m%d).log"
