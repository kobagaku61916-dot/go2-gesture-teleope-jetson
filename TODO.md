# 実装前 TODO リスト

最終更新: 2026-07-08。上から順に消化する（Phase 対応は runbook.md）。

## 準備（コード書く前）— ✅ 完了（2026-07-08）

- [x] GitHub リポジトリ作成・push（`go2-gesture-teleope-jetson`。public）
- [x] Jetson へ配置（`~/go2-gesture-teleop-jetson`。public 化後は git pull 可）
- [x] D435i の USB 接続速度 → **USB3 (5000M)** 確認
- [x] RGB(YUYV) ノード → **/dev/video4** 実測（configs 反映済み）
- [x] venv + **mediapipe 0.10.18** + numpy 1.26.4 導入（setup_jetson.sh 実行成功）
- [x] `mp.solutions.pose` 動作確認（0.10.18 に solutions API あり）

## 移植（architecture.md §5 の一覧どおり）— ✅ 完了（2026-07-08）

- [x] `gesture_mapper.py` + テスト 12 件（変更ゼロ・import パスのみ）
- [x] `dance_detector.py` + テスト 7 件（同上）
- [x] `camera/base.py` / `camera/v4l2_camera.py` / ファクトリ（変更ゼロ）
- [x] `pose_tracker.py` → `src/pose/`（0.10.18 前提化 + `find_visibilities()` 追加）
- [x] `cmd_vel_relay.py` + `action_relay.py` → `src/safety/safety_gate.py` に統合（SIGINT/SIGTERM/**SIGHUP** いずれも 0 送出）
- [x] `sport_mode_bridge_node.py` + `action_bridge.py` → `src/bridge/go2_bridge.py` に統合（enP8p1s0 参加者 1 本）
- [x] tests が Jetson venv で全件パス（**32 件**。※Jetson では `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` が必要 — システム側の壊れた pydantic プラグイン対策）
- [x] ノード 3 本の import スモークテスト（rclpy / unitree_api / mediapipe すべて解決）

## 新規実装 — ✅ 完了（2026-07-08）

- [x] `src/main.py`（gesture_node）: ヘッドレス既定・終了時 0 publish・ラベル変化ログ（Phase 3 用）
- [x] 姿勢推定の**信頼度チェック**（`src/pose/confidence.py`）+ テスト 6 件
- [x] **安定化 debounce**（`src/gesture/debounce.py`。STOP は即時通過）+ テスト 7 件
- [x] **低速モード**（既定 true。0.2/0.3 強制。解除は `--no-low-speed` か configs）
- [x] `configs/params.yaml`
- [x] `scripts/run_gesture_node.sh` / `run_safety_gate.sh` / `run_go2_bridge.sh`
- [x] `scripts/setup_jetson.sh`

## 検証（runbook.md の Phase 1〜4。Go2 は動かさない）

- [x] Phase 1: カメラ取得 fps 実測 → **30.0fps**（requirements §5 記入済み）
- [x] Phase 2: Pose 推論 fps / CPU 実測 → **12.9fps / 1.1 コア（18%）**。complexity 1 を既定に（同 §5.1）
- [x] Phase 3: 全ラベルのログ確認（2026-07-08 完了）— STOP / FORWARD / BACKWARD / TURN-LEFT / TURN-RIGHT / NO BODY / DANCE すべて実機カメラ（Go2 搭載 D435i）で確認。DANCE! 発火 4 回。調整: dance.max_interval_sec 2.0→**3.0**（13fps 環境では 2.0 だとスワップが繋がらない）
- [ ] Phase 4: safety_gate 貫通・クランプ・watchdog・action ゲート拒否/通過を echo で確認

## 実機（Phase 5〜6。人間の安全確認つき）

- [ ] Phase 5 事前チェック（runbook §1）→ 低速 C 節シーケンス完走 → watchdog 実機確認 → 段階速度表で昇速
- [ ] Phase 6: hello → dance1 の経路確認
- [ ] 実測値・トラブルを runbook / requirements に反映して commit

## 保留（動いてから）

- [ ] launch 一発起動 / systemd 化
- [ ] RealSense 深度の活用（距離ベースの安全停止など。ros-humble-realsense2-camera 導入済み）
- [ ] 旧リポ側 README に「後継リポジトリ」の案内を追記
