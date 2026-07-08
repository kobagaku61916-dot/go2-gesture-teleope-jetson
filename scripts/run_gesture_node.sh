#!/usr/bin/env bash
# [Jetson] gesture_node（認識）。DDS: lo 単独 + Peer 127.0.0.1。
# tmux 内で起動すること（runbook.md §2）。引数は src/main.py にそのまま渡る
# （例: ./run_gesture_node.sh --enable-action --display）。
set -e
cd "$(dirname "$0")/.."
mkdir -p logs
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="default"/></Interfaces></General><Discovery><Peers><Peer address="127.0.0.1"/></Peers><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>'
ros2 daemon stop >/dev/null 2>&1 || true
echo "[gesture_node] camera -> pose -> gesture -> /cmd_vel, /go2_action  起動"
exec venv/bin/python3 -m src.main "$@" 2>&1 | tee -a "logs/gesture_node_$(date +%Y%m%d).log"
