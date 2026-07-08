#!/usr/bin/env bash
# [Jetson] safety_gate（クランプ+watchdog+アクションゲート）。DDS: lo 単独 + Peer 127.0.0.1。
# tmux 内で起動すること（runbook.md §2）。
set -e
cd "$(dirname "$0")/.."
mkdir -p logs
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="default"/></Interfaces></General><Discovery><Peers><Peer address="127.0.0.1"/></Peers><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>'
ros2 daemon stop >/dev/null 2>&1 || true
echo "[safety_gate] /cmd_vel -> clamp+watchdog -> /cmd_vel_robot / action gate  起動"
exec venv/bin/python3 -m src.safety.safety_gate 2>&1 | tee -a "logs/safety_gate_$(date +%Y%m%d).log"
