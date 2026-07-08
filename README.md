# go2-gesture-teleop-jetson

**Jetson Orin Nano Super 上で、RealSense D435i による人間の姿勢推定から Unitree Go2 への動作指令までを完結させる**構成のリポジトリ。

> 位置づけ: [go2_gesture_teleop_ros2](https://github.com/kobagaku61916-dot/go2_gesture_teleop_ros2)（Desktop PC で認識 → Wi-Fi 経由で Jetson へ指令）の後継・可搬構成。
> 旧構成の「案C: 全処理 Jetson」（architecture.md 参照）を実現し、**制御ループから Desktop–Jetson 間 Wi-Fi を排除**して安定化する。
> 旧リポで実証済みの判定ロジック・安全機構・運用ノウハウ（トラブル記録 T1〜T4）を最大限再利用する。

**ステータス: Phase 3 完了（2026-07-08）— 全ジェスチャーラベル + ダンス発火を Go2 搭載カメラで確認。次: Phase 4 仮想コマンド確認（[runbook.md](runbook.md)）**

---

## 処理系

```
RealSense D435i（Jetson に USB 接続済み・実測確認済み）
  ↓ OpenCV / V4L2（RGB=YUYV ノードを直接オープン。pyrealsense2 非依存）
MediaPipe Pose（aarch64 wheel の都合で 0.10.18 系）
  ↓ 全身ランドマーク
gesture_mapper / dance_detector（旧リポの純関数をそのまま移植・単体テスト 19 件付き）
  ↓ /cmd_vel (Twist) ・ /go2_action (String)
safety_gate + watchdog（速度クランプ 0.6/0.8・watchdog 0.5s・アクション許可リスト+cooldown+静止確認）
  ↓ /cmd_vel_robot ・ /go2_action_robot   [すべて loopback DDS]
Go2 command bridge（api_id 構築。Move=1008 / Dance1=1022 等・二重許可リスト）
  ↓ /api/sport/request  [enP8p1s0 = 192.168.123.x]
Unitree Go2
```

- 認識・判定・安全制御・指令送信のすべてが Jetson 内で完結する
- Desktop PC は開発・監視端末としてのみ使用（制御経路に入らない）
- 詳細設計・プロセス分割・DDS 構成は [architecture.md](architecture.md)
- Jetson 実機の要件・実測値は [requirements-jetson.md](requirements-jetson.md)
- 起動・検証・緊急停止の運用手順は [runbook.md](runbook.md)
- 実装前の作業一覧は [TODO.md](TODO.md)

## 段階検証計画（いきなり Go2 を動かさない）

| Phase | 内容 | Go2 |
|---|---|---|
| 1 | RealSense 映像取得確認（V4L2 ノード実測・fps 計測） | 動かさない |
| 2 | MediaPipe Pose 推論確認（0.10.18・fps/レイテンシ実測） | 動かさない |
| 3 | ジェスチャー判定のログ出力（判定ロジックの実機等価性確認） | 動かさない |
| 4 | 仮想コマンド出力（/cmd_vel を publish、bridge は起動しない） | 動かさない |
| 5 | safety_gate / watchdog 経由で Go2 に**低速**指令（0.2/0.3 から段階表どおり） | 低速で動く |
| 6 | dance / special action 系の拡張 | 動く |

## 安全原則（旧リポから継承・変更しない）

1. Go2 へ届く速度指令は**必ず safety_gate（クランプ+watchdog）を通す**。bridge 直結禁止
2. **bridge の起動は人間のみ**が安全確認のうえ実行する。自動起動させない
3. アクション系（dance 等）は**二重許可リスト + cooldown + 静止確認**を通す。危険動作（FrontFlip 等）は対応表に載せない
4. 各プロセスは tmux 内で起動する（SSH 切断による SIGHUP 死を防ぐ。旧リポ T1）
5. 緊急停止の優先順位: ①STOP 姿勢 ②視野外へ ③認識ノード停止 ④safety_gate 停止 ⑤純正リモコン/アプリ ⑥電源
6. 初回・構成変更後は必ず低速（0.2/0.3）から段階表で検証する

## リポジトリ構成

```
go2-gesture-teleop-jetson/
├── README.md               # 本ファイル
├── architecture.md         # 設計・プロセス分割・DDS 構成・移植計画
├── requirements-jetson.md  # Jetson 実機要件（実測値ベース）
├── runbook.md              # 起動・検証・緊急停止の運用手順
├── TODO.md                 # 実装前 TODO リスト
├── src/
│   ├── camera/             # V4L2 カメラ抽象化層（旧リポから移植）
│   ├── pose/               # MediaPipe Pose ラッパ（0.10.18 対応）
│   ├── gesture/            # gesture_mapper / dance_detector（純関数・移植）
│   ├── safety/             # safety_gate（クランプ+watchdog）/ action_gate（許可リスト）
│   ├── bridge/             # Go2 command bridge（api_id 構築）
│   └── main.py             # 認識ノード本体（camera→pose→gesture→publish）
├── configs/                # params.yaml（カメラ・しきい値・速度・安全設定）
├── scripts/                # 起動スクリプト（DDS URI 込み）・セットアップ
└── tests/                  # 単体テスト（gesture_mapper 12 件 + dance_detector 7 件を移植）
```
