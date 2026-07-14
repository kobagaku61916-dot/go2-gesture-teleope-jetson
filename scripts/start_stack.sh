#!/usr/bin/env bash
# [Jetson] 認識+安全ゲートのスタックを tmux 内に一発構築する（電源投入後はまずこれ）。
#   - tmux セッション go2j（gesture/safety/bridge/monitor）を作成
#   - safety_gate と gesture_node を起動（既定: 追従モード + YOLO）
#   - ※ go2_bridge は起動しない（実機が動くため人間が bridge 窓で手動起動する）
# 使い方:
#   ~/go2-gesture-teleop-jetson/scripts/start_stack.sh            # 追従(YOLO)
#   ~/go2-gesture-teleop-jetson/scripts/start_stack.sh --teleop   # ジェスチャーテレオペ
set -e
REPO=~/go2-gesture-teleop-jetson

MODE_ARGS="--follow --backend yolo"
if [ "$1" = "--teleop" ]; then
    MODE_ARGS="--enable-action"
fi

# カメラ確認（USB 再列挙で番号が変わっていたら中断して知らせる）
if ! v4l2-ctl -d /dev/video4 --list-formats 2>/dev/null | grep -q YUYV; then
    echo "ERROR: /dev/video4 が RGB(YUYV) ではありません。RealSense の接続と"
    echo "       'v4l2-ctl --list-formats -d /dev/videoN' で YUYV ノードを確認してください。"
    exit 1
fi

tmux has-session -t go2j 2>/dev/null || {
    tmux new-session -d -s go2j -n gesture
    for w in safety bridge monitor; do tmux new-window -t go2j -n "$w"; done
}
tmux send-keys -t go2j:safety "$REPO/scripts/run_safety_gate.sh" C-m
sleep 6
tmux send-keys -t go2j:gesture "$REPO/scripts/run_gesture_node.sh $MODE_ARGS" C-m
sleep 14

echo "--- 起動確認 ---"
ps aux | grep -E "src\.main|src\.safety" | grep -v grep | grep -v "bash -c" \
    | awk '{print "  OK:", $12, $13}'
echo
echo "次: 安全確認のうえ bridge を人間が起動する:"
echo "  tmux attach -t go2j   (Ctrl-b 2 で bridge 窓)"
echo "  $REPO/scripts/run_go2_bridge.sh"
