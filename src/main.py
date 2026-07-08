"""gesture_node（認識ノード本体）— 設計フェーズのスタブ.

パイプライン（architecture.md §2 プロセス1）:
    camera(V4L2/RGB) → pose(MediaPipe 0.10.18 + 信頼度チェック)
    → gesture(gesture_mapper / dance_detector + debounce)
    → /cmd_vel (Twist) ・ /go2_action (String) を lo DDS へ publish

実装方針:
- 旧リポ gesture_teleop_node.py の骨格（終了時 0 publish・フレーム失敗時 0 送出・
  SIGINT/SIGTERM ハンドラ）を踏襲する
- ヘッドレス既定（display は開発時のみ）
- パラメータは configs/params.yaml から

TODO(Phase 3): 実装（TODO.md「新規実装」参照）
"""

import sys


def main() -> int:
    print("gesture_node: 未実装（設計フェーズ）。TODO.md と runbook.md Phase 1-3 を参照。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
