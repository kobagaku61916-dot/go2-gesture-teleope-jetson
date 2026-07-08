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

⚠️ 未確認: D435i が USB3 で繋がっているか（`lsusb -t` で要確認）。USB2 でも RGB 640x480@30fps は通る見込みだが Phase 1 で実測する。

## 2. ソフトウェア（実測済み ✅ / 要導入 ⬜）

| 項目 | 状態 | 備考 |
|---|---|---|
| ROS2 Humble | ✅ 導入済み | relay/bridge の運用実績あり |
| rmw_cyclonedds_cpp | ✅ 導入済み（apt 1.3.4） | `~/.bashrc` に設定済み |
| unitree_api メッセージ | ✅ 導入済み | `~/go2_guide_dog/unitree_ros2`（bridge が依存） |
| OpenCV (python) | ✅ **4.11.0**（システム） | Desktop と同一メジャー。venv は `--system-site-packages` で共有 |
| Python | ✅ 3.10.12 | mediapipe cp310 wheel に一致 |
| librealsense2 / realsense2_camera | ✅ ros-humble 版導入済み | 当面未使用（V4L2 直接方式）。深度を使う段で活用 |
| **mediapipe** | ⬜ 未導入 | **0.10.18 を pip 導入**（下記 §3） |
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

| 項目 | 目標 | 実測値 |
|---|---|---|
| カメラ取得 fps（640x480） | 30fps | （Phase 1 で記入） |
| Pose 推論込みループ fps（complexity 1） | ≥ 15fps | （Phase 2 で記入） |
| 同（complexity 0） | ≥ 25fps | （Phase 2 で記入） |
| ジェスチャー→ /cmd_vel レイテンシ | 体感 0.5s 未満 | （Phase 3-5 で記入） |
| CPU 使用率（推論中） | 全体の 70% 未満（ロボット系と共存のため） | （Phase 2 で記入） |

- watchdog(0.5s) の成立要件は publish ≥ 5Hz 程度なので、10fps でも制御は成立する。fps はレイテンシ体感で判断する
- complexity と解像度はこの実測を見て configs で決める
