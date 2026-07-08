# scripts

| スクリプト | 状態 | 内容 |
|---|---|---|
| `setup_jetson.sh` | ✅ 作成済み | venv + mediapipe 0.10.18 導入・動作確認 |
| `run_gesture_node.sh` | ⬜ 実装時に作成 | DDS: **lo 単独** + Peer 127.0.0.1。venv python で src/main.py |
| `run_safety_gate.sh` | ⬜ 実装時に作成 | DDS: **lo 単独** + Peer 127.0.0.1 |
| `run_go2_bridge.sh` | ⬜ 実装時に作成 | DDS: **enP8p1s0 単独** + Peer 127.0.0.1（旧 T2 の実証構成。lo を束ねない） |

共通方針:
- `export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + 各 CYCLONEDDS_URI
- 起動は必ず tmux 内・`2>&1 | tee -a logs/<name>_$(date +%Y%m%d).log` 付き（runbook.md §2）
- URI の実証済みパターンは旧リポ `jetson/run_action_relay.sh` / `run_action_bridge.sh` を参照
