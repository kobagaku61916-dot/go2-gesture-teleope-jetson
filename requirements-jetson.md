# Jetson 実機要件（実測値ベース）

最終更新: 2026-07-08（SSH 読み取りで実測。値が変わったら更新すること）

---

## 1. ハードウェア（実測済み ✅）

| 項目 | 実測値 | 判定 |
|---|---|---|
| 機体 | NVIDIA **Jetson Orin Nano Engineering Reference Developer Kit Super** | ✅ |
| OS/BSP | Ubuntu 22.04 / L4T **R36.4.7**（JetPack 6 系） | ✅ |
| CPU | 6 コア aarch64 | ✅ MediaPipe CPU 推論に十分な見込み（Phase 2 で実測） |
| RAM | 7.4GiB（空き 5.6GiB） | ✅ |
| ストレージ | NVMe 233GB（空き 97GB） | ✅ |
| 電力モード | **25W**（nvpmodel） | ✅ このモードを維持する（省電力モードだと推論が遅くなる） |
| カメラ | **RealSense D435i が USB 接続済み**（`lsusb` 8086:0b3a、`/dev/video0〜5`） | ✅ |
| ネットワーク | wlx…=192.168.11.10（Wi-Fi, 監視用）/ enP8p1s0=192.168.123.18（Go2 内部網） | ✅ |

✅ 2026-07-08 実測: D435i は **USB3（5000M）接続**（`lsusb -t`）。RGB ノードは **/dev/video4（YUYV）**（video0=深度 Z16、video2=IR GREY。Desktop と同配置）。

## 2. ソフトウェア（実測済み ✅ / 要導入 ⬜）

| 項目 | 状態 | 備考 |
|---|---|---|
| python3.10-venv | ✅ 導入済み（2026-07-08 apt） | |
| ROS2 Humble | ✅ 導入済み | relay/bridge の運用実績あり |
| rmw_cyclonedds_cpp | ✅ 導入済み（apt 1.3.4） | `~/.bashrc` に設定済み |
| unitree_api メッセージ | ✅ 導入済み | `~/go2_guide_dog/unitree_ros2`（bridge が依存） |
| OpenCV (python) | ✅ **4.11.0**（システム） | Desktop と同一メジャー。venv は `--system-site-packages` で共有 |
| Python | ✅ 3.10.12 | mediapipe cp310 wheel に一致 |
| librealsense2 / realsense2_camera | ✅ ros-humble 版導入済み | 当面未使用（V4L2 直接方式）。深度を使う段で活用 |
| **mediapipe** | ✅ **0.10.18 導入済み**（2026-07-08, venv） | numpy **1.26.4** / opencv-contrib 4.11.0.86 と共存（旧リポと同一の組）。`mp.solutions.pose` 動作確認済み |
| pyrealsense2 (python) | ❌ 導入しない | V4L2 + OpenCV 方式のため不要（設計方針） |
| tmux | ✅ 3.2a 導入済み | 運用必須（T1 対策） |
| インターネット接続 | ✅ | pip/apt 可 |

## 3. MediaPipe バージョン方針（重要）

- 旧リポの固定 **0.10.21 には Linux aarch64 wheel が存在しない**（PyPI 全バージョン走査で確認済み・2026-07-08）
- Linux aarch64 wheel が存在する最新は **0.10.18**（cp310 あり）→ **これを固定する**
- 0.10.18 は legacy solutions API（`mp.solutions.pose`）を**含む**世代（API 除去は 0.10.35 以降）
  → `pose_tracker.py` は小差分で移植できる見込み。**判定ロジックの等価性は移植済み単体テスト 19 件で保証する**
- numpy / opencv との組み合わせは Phase 2 で実測し、動いた組み合わせを本ファイルに固定値として記録すること
  （旧リポは mediapipe 0.10.21 + opencv 4.11.0.86 + numpy 1.26.4 の組で固定していた。numpy は 2 系を避けるのが無難）

## 4. セットアップ手順（案。scripts/setup_jetson.sh に実装予定）

```bash
cd ~/go2-gesture-teleop-jetson
python3 -m venv venv --system-site-packages   # rclpy・cv2 をシステムから見せる
source venv/bin/activate
pip install "mediapipe==0.10.18" "numpy<2"
python3 -c "import mediapipe as mp; print(mp.__version__); mp.solutions.pose"  # solutions API 確認
source /opt/ros/humble/setup.bash
python3 -c "import rclpy, cv2; print('rclpy/cv2 OK', cv2.__version__)"
```

## 5. 性能目標（Phase 1/2 で実測して埋める）

| 項目 | 目標 | 実測値（2026-07-08） |
|---|---|---|
| カメラ取得 fps（640x480） | 30fps | ✅ **30.0 fps**（/dev/video4, V4L2 直接） |
| Pose 推論込みループ fps（complexity 1） | ≥ 15fps | **12.9 fps**（60 フレーム計測） |
| 同（complexity 0） | ≥ 25fps | **12.9 fps**（complexity 1 と同値 → ボトルネックは推論以外の可能性。§5.1） |
| ジェスチャー→ /cmd_vel レイテンシ | 体感 0.5s 未満 | （Phase 3-5 で記入。13fps ≈ 実効遅延 0.15〜0.25s 見込み） |
| CPU 使用率（推論中） | 全体の 70% 未満（ロボット系と共存のため） | ✅ **1.1 コア相当 = 6 コア中 18%**（大きな余裕） |

### 5.1 実測所感

- 12.9fps は watchdog 成立要件（≥5Hz）に対し十分。目標 15fps には僅かに届かないが**制御は成立**する
- complexity 0 と 1 が同速だったのは想定外（lite モデルなら速いはず）。CPU が 1.1 コアしか使われていないことと合わせ、
  ボトルネックは推論スレッド数 or カメラ read の同期にある可能性 → **判定精度優先で complexity 1 を既定**とし、
  高速化（スレッド数・read の非同期化）は動いてから Phase 6 以降の最適化課題とする
- watchdog(0.5s) の成立要件は publish ≥ 5Hz 程度なので、10fps でも制御は成立する。fps はレイテンシ体感で判断する
