#!/usr/bin/env bash
# [Jetson] venv 作成 + mediapipe 0.10.18 導入 + 動作確認（requirements-jetson.md §4）
set -e
cd "$(dirname "$0")/.."
python3 -m venv venv --system-site-packages   # rclpy・cv2(4.11) をシステムから見せる
source venv/bin/activate
pip install "mediapipe==0.10.18" "numpy<2"
python3 - <<'EOF'
import mediapipe as mp
import numpy, cv2
print("mediapipe", mp.__version__, "/ numpy", numpy.__version__, "/ cv2", cv2.__version__)
mp.solutions.pose  # legacy solutions API が存在すること
print("solutions API OK")
EOF
source /opt/ros/humble/setup.bash
python3 -c "import rclpy; from geometry_msgs.msg import Twist; print('rclpy OK')"
echo "setup 完了。次: TODO.md の Phase 1（カメラノード実測）へ"
