# アーキテクチャ設計（Jetson 単体完結構成）

最終更新: 2026-07-08（設計フェーズ。実装前）

旧リポ [go2_gesture_teleop_ros2](https://github.com/kobagaku61916-dot/go2_gesture_teleop_ros2) の実測知見
（Phase 4/5 実機検証・troubleshooting T1〜T4・dance_action 実装）を前提とする。

---

## 1. 設計の動機

旧構成は「Desktop で認識 → Wi-Fi DDS → Jetson relay → bridge → Go2」だった。実運用で判明した弱点:

- Desktop–Jetson 間 Wi-Fi/DDS の不安定（discovery 断・SSH 切断で relay 巻き添え = T1/T4）が**制御経路そのもの**に乗っていた
- カメラ位置が Desktop に拘束され、Go2 の歩行に追従できない（可搬性なし）

本構成では認識〜指令送信を Jetson 内で完結させ、**制御ループから Wi-Fi を完全に排除**する。
Desktop は SSH/tmux での起動操作・監視・開発のみに使う（切れても制御は継続する）。

## 2. プロセス分割（3 プロセス構成）

旧リポ T10/T2 の実測知見「**複数 NIC を 1 つの DDS 参加者に束ねると discovery が壊れる**」に従い、
Jetson 内でもプロセスを NIC 境界で分割する。旧構成の relay/bridge 分離パターンをそのまま踏襲する。

```
[プロセス1: gesture_node（src/main.py）]                DDS: lo 単独
  V4L2 カメラ → MediaPipe Pose → 信頼度チェック
  → gesture_mapper（速度）/ dance_detector（アクション）→ 判定安定化
  → /cmd_vel (Twist, ~15-30Hz) ・ /go2_action (String, 発火時のみ)

[プロセス2: safety_gate（src/safety/）]                 DDS: lo 単独
  /cmd_vel → クランプ(MAX 0.6/0.8) + watchdog(0.5s 途絶で 0) → /cmd_vel_robot
  /go2_action → 許可リスト + cooldown(15s) + 静止確認(1s) → /go2_action_robot
  終了時・シグナル受信時は 0 を送出してから落ちる

[プロセス3: go2_bridge（src/bridge/）]                  DDS: enP8p1s0 単独 + Peer 127.0.0.1
  /cmd_vel_robot → Request(api_id=1008 Move, {x,y,z})
  /go2_action_robot → Request(対応表: dance1=1022, dance2=1023, hello=1016,
                              stop_move=1003, balance_stand=1002。parameter 空)
  → /api/sport/request → Go2 (192.168.123.161)
```

- プロセス 1↔2↔3 の通信はすべて **loopback ユニキャスト**（`NetworkInterface lo` + `Peer 127.0.0.1` + `ParticipantIndex auto`）。旧構成で実証済みのパターン
- プロセス 3 の「enP8p1s0 単独 + Peer 127.0.0.1」は旧 T2 で実測確立した構成（lo を束ねると Go2 を発見できない。同一ホスト間は lo なしでもユニキャストで届く）
- **watchdog を認識プロセスから分離**する理由: 認識側（カメラ/MediaPipe）がハングしても、独立プロセスの watchdog が 0.5s で停止指令を出せる
- 旧構成との差分: relay の購読相手が「Wi-Fi 越しの Desktop」から「同一ホストの gesture_node」に変わるだけ。safety_gate/bridge のコードは旧 Jetson 資産とほぼ同一

## 3. 環境・実行系

- 実行: venv（`--system-site-packages`。システムの OpenCV 4.11 / rclpy を見せる）+ mediapipe **0.10.18**（aarch64 wheel の最新。詳細は [requirements-jetson.md](requirements-jetson.md)）
- ROS2 Humble + CycloneDDS（`ROS_DOMAIN_ID=0` / `rmw_cyclonedds_cpp`。Jetson 運用実績どおり）
- 各プロセスは tmux セッション内で起動（scripts/ の run スクリプト。T1 対策）
- 認識ノードは**ヘッドレス既定**（display は開発時のみ。Jetson に :0 はあるが常用しない）

## 4. 安全設計（旧リポから継承 + 本構成での追加）

### 継承（実証済み・変更しない）
| 層 | 機構 |
|---|---|
| safety_gate | 速度クランプ MAX_LINEAR=0.6 / MAX_ANGULAR=0.8、watchdog 0.5s、終了時 0 送出 |
| action gate | 許可リスト（dance1/dance2/hello/stop_move/balance_stand）、cooldown 15s、/cmd_vel 静止 1s 確認、緊急系（stop_move/balance_stand）はバイパスで即通し |
| bridge | 二重許可リスト（対応表にない名前は破棄）。**bridge 自体の起動は人間のみ** |
| 運用 | 起動順序 = 安全装置（bridge 最後）、宣言制テスト、段階速度表、tmux 運用 |

### 本構成での追加（新規実装）
| 追加 | 内容 | 理由 |
|---|---|---|
| 姿勢推定の信頼度チェック | MediaPipe ランドマークの visibility が下限未満のフレームは NO BODY 扱い（0 指令） | Jetson カメラは Go2 搭載で振動・照明変化が大きい |
| ジェスチャー判定の安定化 | N フレーム連続一致（既定 3）ではじめて非ゼロ指令を出す debounce | 誤検出 1 フレームが即 Go2 の動きになるのを防ぐ（旧リポで 1 フレームの誤指令を実測） |
| 低速モード | configs で `low_speed_mode: true` の間は 0.2/0.3 に強制 | Phase 5 初期検証・デモ安全用 |

## 5. 移植計画（既存ファイルの棚卸し）

### ✅ そのまま再利用（純関数・実証済み。単体テストごと移植）

| 旧ファイル（go2_gesture_teleop_ros2） | 移植先 | 備考 |
|---|---|---|
| `go2_gesture_teleop/gesture_mapper.py` | `src/gesture/gesture_mapper.py` | 純関数。テスト 12 件ごと。**変更ゼロ** |
| `go2_gesture_teleop/dance_detector.py` | `src/gesture/dance_detector.py` | 純関数（交互腕伸ばし方式）。テスト 7 件ごと。**変更ゼロ** |
| `go2_gesture_teleop/camera/base.py` | `src/camera/base.py` | 抽象インターフェース。変更ゼロ |
| `go2_gesture_teleop/camera/v4l2_camera.py` | `src/camera/v4l2_camera.py` | 変更ゼロ見込み（device 番号は configs で） |
| `test/test_gesture_mapper.py` / `test_dance_detector.py` | `tests/` | 変更ゼロ（import パスのみ） |

### ✅ ほぼ再利用（Jetson 上の実績資産をリポジトリに取り込む）

| 旧ファイル（Jetson ~/gesture_bridge ほか） | 移植先 | 変更点 |
|---|---|---|
| `cmd_vel_relay.py`（クランプ+watchdog） | `src/safety/cmd_vel_gate.py` | ロジック変更なし。SIGHUP/SIGTERM でも 0 送出するハンドラ追加（T1 の教訓） |
| `action_relay.py`（許可リスト+cooldown+静止確認） | `src/safety/action_gate.py` | 同上 |
| `sport_mode_bridge_node.py`（go2_guide_dog 由来, Move 1008） | `src/bridge/go2_bridge.py` | action_bridge と統合し 1 プロセスに（両方 enP8p1s0 参加者のため統合可） |
| `action_bridge.py`（旧リポ jetson/ に参照コピーあり） | 同上 | 同上 |

### 🔧 書き直し・新規実装

| 対象 | 内容 | 理由 |
|---|---|---|
| `gesture_teleop_node.py` → `src/main.py` | ノード骨格は流用しつつ書き直し | ヘッドレス既定 / 信頼度チェック・debounce の追加 / lo 単独 URI 前提 / display は cv2 ウィンドウでなく画像保存 or 低頻度ログも選べるように |
| `pose_tracker.py` → `src/pose/pose_tracker.py` | ほぼ流用だが **mediapipe 0.10.18 で API 差分を要確認** | 0.10.21→0.10.18 のダウングレード（solutions API は 0.10.18 に存在するため小差分見込み） |
| 信頼度チェック | 新規（`src/pose/` 内） | 上記 §4 |
| debounce | 新規（`src/gesture/` 内・純関数で） | 上記 §4 |
| `scripts/run_*.sh` | 新規 3 本（gesture_node / safety_gate / go2_bridge） | lo 単独・enP8p1s0 単独の URI。旧 run_relay.sh / run_bridge.sh の URI パターンを流用 |
| `scripts/setup_jetson.sh` | 新規 | venv 作成 + mediapipe 0.10.18 導入 + 動作確認 |
| `configs/params.yaml` | 旧 params.yaml を基に再構成 | camera.device は Phase 1 の実測で決定。安全系パラメータを追加 |

### ❌ 移植しないもの

- Desktop↔Jetson の Wi-Fi ユニキャスト DDS 設定（wlx 側 URI・Peer 192.168.11.2）— 制御経路から排除するのが本構成の目的
- Desktop 側の venv / RealSense 設定（Desktop はもはや制御に関与しない）
- `teleop.launch.py`（launch 化は動いてから。Phase 6 以降）

## 6. トピック一覧（Jetson 内で完結）

| トピック | 型 | publish | subscribe | 備考 |
|---|---|---|---|---|
| /cmd_vel | Twist | gesture_node | safety_gate | 生の速度指令（クランプ前） |
| /cmd_vel_robot | Twist | safety_gate | go2_bridge | クランプ+watchdog 後 |
| /go2_action | String | gesture_node | safety_gate | 抽象アクション名 |
| /go2_action_robot | String | safety_gate | go2_bridge | ゲート通過済み |
| /api/sport/request | unitree_api/Request | go2_bridge | Go2 本体 | api_id + parameter |

トピック名を旧構成と同一にしてあるため、**旧リポの監視ノウハウ（probe URI・daemon stop の作法 = T4）がそのまま使える**。
