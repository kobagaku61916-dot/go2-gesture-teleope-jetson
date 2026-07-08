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
- [x] Phase 4: 完了（2026-07-08）— 実測結果:
  - 無人ゼロ貫通（131 msgs 全ゼロ・~11Hz）/ クランプ（1.0→**0.6**, 2.0→**0.8**）/ **watchdog 発火 ×2**
  - ジェスチャー → ±指令貫通（FORWARD 0.2×84 / BACKWARD / TURN±0.3。debounce 経由）
  - ダンス発火 → ゲート**通過 ×2**、**拒否(未許可: backflip)**、**拒否(移動指令 1s 内)** を live 実証
  - 調整 3 件: `min_visibility` 0.5→**0.3**（腕上げで手首 visibility が下がり FORWARD⇄NO BODY ちらつき）/
    `no_body_grace_sec` **0.7 追加**（欠損 1 フレームでダンスチェーン全消失）/ `max_interval_sec` → 3.0（Phase 3）

## 実機（Phase 5〜6。人間の安全確認つき）— ✅ 完了（2026-07-08）

- [x] Phase 5: 低速（0.2/0.3）C 節完走 → **最高速（0.6/0.8 = クランプ上限）で全 4 方向+複合を完走** → 既定速度化
  - 記録: 低速 FORWARD 0.2×177 / BACKWARD ×64、最高速 ±0.6×100 / ±0.8×91（すべて上限値ちょうどで出力）
  - watchdog 実機発火を複数回確認。NO BODY 自動停止（搭載カメラで移動→視野が外れる→停止）も安全側に機能
- [x] Phase 6: **Greet（右手振り 2 秒 → hello）と Dance1（交互腕伸ばし 5 秒）を実機で複数回成功**
  - 対面ミラー旋回（mirror_turns）を追加（手を出した側と同じ方向へ回る）
  - Greet は当初「両手かざし」→ 実機フィードバックで「右手振り」方式へ変更（wave_detector）
- [x] 実機で見つけた調整の記録: min_visibility 0.3 / no_body_grace 0.7s / dance max_interval 3.0s / wave amplitude 0.18・gap 1.2s /
  **safety_gate は lo 単独では bridge から発見できず wlx+lo が必須**（旧 relay と同構成）/ USB 再列挙で /dev/videoN が変わる

## 人追従モード（2026-07-09 実機成功・改良継続中）

- [x] follow_controller v1（肩幅ベース P 制御）実装 + Phase A 仮想検証（前後・旋回・デッドバンド・後退すべて期待どおり）
- [x] 1.5m 校正: **sw_at_target = 0.105 実測**（幾何推定 0.19 は大幅ズレ — MediaPipe の肩点は物理肩幅より内側）
- [x] 追従モードは視認チェックを**両肩のみ**に（手首要求だと歩行中に検出が切れる）
- [x] **Phase B 実機追従成功**（近づく→1.5m 停止→後退追従→旋回追従→近すぎ後退）
- [x] v2: 距離[m]推定ベース P（遠方で 0.6 に飽和）+ 見失い猶予 0.25s + ローパス緩和 → 「遅い・ぎこちない」を改善
- [ ] **動き出しの遅さの原因調査（次回）**。仮説: ①ローパスがゼロから立ち上がる（起動キック不足）②検出獲得の初期遅延（tracking モードの初回検出）③13fps の制御周期そのもの ④デッドバンド境界のためらい。ログ+記録データ（/tmp/follow_run.txt 等）で切り分け
- [ ] **YOLO 系（YOLOv8/11-pose 等）の試用検討（次回）**: Orin Nano の GPU + TensorRT で BlazePose(CPU 13fps) より高速・高頑健の可能性。検証観点: fps / 検出安定性（遠距離・部分隠れ）/ 33点→17点への gesture_mapper 対応 / CPU/GPU 負荷

## 保留（動いてから）

- [ ] launch 一発起動 / systemd 化
- [ ] RealSense 深度の活用（距離ベースの安全停止など。ros-humble-realsense2-camera 導入済み）
- [ ] 旧リポ側 README に「後継リポジトリ」の案内を追記
