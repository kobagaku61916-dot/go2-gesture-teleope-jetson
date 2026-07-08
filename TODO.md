# 実装前 TODO リスト

最終更新: 2026-07-08。上から順に消化する（Phase 対応は runbook.md）。

## 準備（コード書く前）

- [ ] GitHub リポジトリ作成・push（Desktop から）
- [ ] Jetson へ clone（`~/go2-gesture-teleop-jetson`）
- [ ] `lsusb -t` で D435i の USB 接続速度を確認（USB2/USB3）→ requirements-jetson.md に記録
- [ ] `v4l2-ctl --list-formats` で **RGB(YUYV) ノードを実測特定** → configs/params.yaml に記録
- [ ] venv 作成 + **mediapipe 0.10.18** + numpy<2 導入（scripts/setup_jetson.sh 化）
- [ ] `mp.solutions.pose` が 0.10.18 で動くこと・API 差分の有無を確認（pose_tracker 移植の前提）

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

- [ ] Phase 1: カメラ取得 fps 実測 → requirements-jetson.md §5 に記入
- [ ] Phase 2: Pose 推論 fps / CPU 使用率実測（complexity 0/1 比較）→ 既定値決定
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
