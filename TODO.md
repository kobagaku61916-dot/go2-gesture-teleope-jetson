# 実装前 TODO リスト

最終更新: 2026-07-08。上から順に消化する（Phase 対応は runbook.md）。

## 準備（コード書く前）— ✅ 完了（2026-07-08）

- [x] GitHub リポジトリ作成・push（`go2-gesture-teleope-jetson`。public）
- [x] Jetson へ配置（`~/go2-gesture-teleop-jetson`。public 化後は git pull 可）
- [x] D435i の USB 接続速度 → **USB3 (5000M)** 確認
- [x] RGB(YUYV) ノード → **/dev/video4** 実測（configs 反映済み）
- [x] venv + **mediapipe 0.10.18** + numpy 1.26.4 導入（setup_jetson.sh 実行成功）
- [x] `mp.solutions.pose` 動作確認（0.10.18 に solutions API あり）

## 移植（architecture.md §5 の一覧どおり）

- [ ] `gesture_mapper.py` + テスト 12 件（変更ゼロ・import パスのみ）
- [ ] `dance_detector.py` + テスト 7 件（同上）
- [ ] `camera/base.py` / `camera/v4l2_camera.py`（変更ゼロ見込み）
- [ ] `pose_tracker.py` → `src/pose/`（0.10.18 対応。差分があればここで吸収）
- [ ] Jetson 資産 `cmd_vel_relay.py` / `action_relay.py` → `src/safety/`（SIGHUP/SIGTERM でも 0 送出するハンドラ追加）
- [ ] `sport_mode_bridge_node.py` + `action_bridge.py` → `src/bridge/go2_bridge.py` に統合（enP8p1s0 参加者 1 本化）
- [ ] tests が Jetson venv で全件パス（19 件）

## 新規実装

- [ ] `src/main.py`（gesture_node）: ヘッドレス既定・lo URI 前提・終了時 0 publish
- [ ] 姿勢推定の**信頼度チェック**（visibility 下限。未満は NO BODY 扱い）+ 単体テスト
- [ ] ジェスチャー判定の**安定化 debounce**（N フレーム連続一致で確定。純関数）+ 単体テスト
- [ ] **低速モード**（configs で 0.2/0.3 強制）
- [ ] `configs/params.yaml`（カメラ・しきい値・速度・安全設定・アクション設定）
- [ ] `scripts/run_gesture_node.sh` / `run_safety_gate.sh` / `run_go2_bridge.sh`（DDS URI 込み・tee ログ付き）
- [ ] `scripts/setup_jetson.sh`

## 検証（runbook.md の Phase 1〜4。Go2 は動かさない）

- [x] Phase 1: カメラ取得 fps 実測 → **30.0fps**（requirements §5 記入済み）
- [x] Phase 2: Pose 推論 fps / CPU 実測 → **12.9fps / 1.1 コア（18%）**。complexity 1 を既定に（同 §5.1）
- [ ] Phase 3: 全ラベルのログ確認・debounce の効き確認
- [ ] Phase 4: safety_gate 貫通・クランプ・watchdog・action ゲート拒否/通過を echo で確認

## 実機（Phase 5〜6。人間の安全確認つき）

- [ ] Phase 5 事前チェック（runbook §1）→ 低速 C 節シーケンス完走 → watchdog 実機確認 → 段階速度表で昇速
- [ ] Phase 6: hello → dance1 の経路確認
- [ ] 実測値・トラブルを runbook / requirements に反映して commit

## 保留（動いてから）

- [ ] launch 一発起動 / systemd 化
- [ ] RealSense 深度の活用（距離ベースの安全停止など。ros-humble-realsense2-camera 導入済み）
- [ ] 旧リポ側 README に「後継リポジトリ」の案内を追記
