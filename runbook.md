# 運用手順書（runbook）

最終更新: 2026-07-08（設計フェーズ。Phase を進めるたびに実測値・手順を更新すること）

旧リポの [phase5_safety_checklist.md](https://github.com/kobagaku61916-dot/go2_gesture_teleop_ros2/blob/main/docs/phase5_safety_checklist.md) と
[troubleshooting.md](https://github.com/kobagaku61916-dot/go2_gesture_teleop_ros2/blob/main/docs/troubleshooting.md)（T1〜T4）を前提知識とする。

---

## 0. 大原則

- **go2_bridge の起動 = 実機が動きうる瞬間**。人間が Go2 の横で安全確認のうえ起動する（自動起動禁止）
- すべてのプロセスは **tmux セッション内**で起動（SSH 切断の SIGHUP 対策 = T1）
- テスト開始・終了は**宣言制**（オペレータが宣言してからカメラ前へ）
- Go2 赤ランプが出たら**即中止**。原因確認まで bridge を再起動しない
- 初回・構成変更後は**低速モード（0.2/0.3）**から段階表どおりに

## 1. 段階検証の実施手順

### Phase 1: RealSense 映像取得確認（Go2 不要）
1. `v4l2-ctl --list-formats -d /dev/videoN` で **YUYV が出るノード**を特定（D435i は 6 ノード。RGB は 1 つだけ）→ configs に記録
2. OpenCV で 640x480 取得・fps 計測 → requirements-jetson.md §5 に記入
3. USB 接続速度確認: `lsusb -t`（USB2 なら fps 上限に注意）

### Phase 2: MediaPipe Pose 推論確認（Go2 不要）
1. `scripts/setup_jetson.sh` で venv + mediapipe 0.10.18 導入
2. `tests/` の単体テスト 19 件がパスすること（判定ロジックの等価性確認）
3. カメラ + Pose 推論ループの fps / CPU 使用率を実測 → requirements-jetson.md §5 に記入
4. complexity 0/1 を比較し configs の既定を決める

### Phase 3: ジェスチャー判定のログ出力（Go2 不要）
1. gesture_node をログ出力モードで起動（publish なし or /cmd_vel のみ）
2. カメラ前で FORWARD / BACKWARD / TURN / STOP / NO BODY / ダンスの各ラベルを確認
3. 信頼度チェック・debounce の効き（1 フレーム誤検出が指令にならないこと）を確認

### Phase 4: 仮想コマンド出力（Go2 不要・bridge 起動しない）
1. gesture_node + safety_gate を起動（**go2_bridge は起動しない**）
2. `/cmd_vel_robot` を echo: 無人でゼロ安定 → ジェスチャーで ±値 → クランプ・watchdog（node を止めて 0.5s 後にゼロ 1 発）を確認
3. ダンス → `/go2_action_robot` に通過ログが出ること・ゲート拒否（cooldown/移動中）を確認

### Phase 5: 実機・低速（人間の安全確認つき）
1. 事前確認: Go2 standing / バッテリー / **純正リモコンを監視者が保持** / 周囲 5m×5m / カメラ全身
2. 起動順序（変えない）: ①safety_gate → ②gesture_node（**低速モード**）→ ③無人ゼロ確認 → ④**人間が** go2_bridge 起動 → ⑤30 秒無動作を目視確認 → ⑥宣言してテスト
3. C 節シーケンス: STOP30s → 前進 1-2s → STOP → 後退 → STOP → 左旋回 → STOP → 右旋回 → STOP
4. watchdog 実機確認は最後（前進中に gesture_node を停止 → 0.5s で Go2 停止）
5. 速度は段階表: 0.2/0.3 → 0.3/0.4 → 0.4/0.5 → 0.6/0.8。**速度変更時は bridge を止めてから** node を入れ替える

### Phase 6: dance / special action
1. まず hello（低リスク）で経路確認 → dance1 本番（旧リポで実証済みの手順）
2. 新アクション追加時は許可リスト（safety_gate と bridge の両方）+ 公式 api_id 確認 + 単体で実機検証 → 結果を記録

## 2. tmux 構成（案）

```bash
tmux new -s go2j -n gesture    # window 0: gesture_node（認識）
tmux new-window -t go2j -n safety    # window 1: safety_gate
tmux new-window -t go2j -n bridge    # window 2: go2_bridge（人間のみ起動）
tmux new-window -t go2j -n monitor   # window 3: echo/htop 等の監視
# 各 window で scripts/run_*.sh を tee 付きで起動（ログは logs/ に日付付きで保存）
```

## 3. 停止手順（テスト終了時）

1. オペレータ STOP 姿勢 → Go2 静止を目視
2. go2_bridge を Ctrl+C（window 2）
3. gesture_node を Ctrl+C（終了時 0 publish）
4. safety_gate を Ctrl+C（終了時 0 publish）

## 4. 緊急停止（優先順位つき・全員が事前に把握）

| 優先 | 手段 | 効果 |
|---|---|---|
| 1 | STOP 姿勢（両手下げ） | 即 0 指令 |
| 2 | カメラ視野から出る（NO BODY） | 即 0 指令 |
| 3 | gesture_node Ctrl+C | 終了時 0 + 0.5s 後 watchdog |
| 4 | safety_gate Ctrl+C | 終了時 0 を /cmd_vel_robot へ |
| 5 | 純正リモコン / アプリ停止 | ソフト経路と独立（**ダンス等アクション中はこれが最優先**） |
| 6 | Go2 電源 | 最終手段 |

## 5. 監視の作法（旧リポ T4 の教訓）

- probe（`ros2 topic echo` 等）の前に**毎回 `ros2 daemon stop`**（古い daemon が偽陰性を作る）
- `pgrep -f` には**必ず `| grep -v "bash -c"`**（自己マッチで「稼働中」に見える）
- リモートで echo をパイプ集計するとハングする → **一時ファイルに落としてから集計**
- 「NO MESSAGES」を見ても即異常と断定しない。**別の観測点から再確認**してから結論を出す
- lo 上のトピック監視 URI: `NetworkInterface lo` + `Peer 127.0.0.1` + `ParticipantIndex auto`

## 6. 既知の Jetson 固有注意

- Go2 の電源断 = Jetson の電源断（給電同源）。**再起動のたびに tmux セッションは消える**ので毎回作り直す
- 赤ランプ・通信異常の記録項目: 発生時刻 / 直前のジェスチャー / /cmd_vel_robot / /api/sport/request / Wi-Fi 状態
- USB 抜き差しで /dev/videoN の番号が変わることがある（YUYV ノードを確認してから起動）
