"""follow_controller.FollowController の単体テスト（rclpy 不要）.

座標系は他テストと同じ（画面左上原点・幅 640）。target_sw=0.19 に対し
肩幅ピクセルで距離を模擬する（sw = 肩幅px / 640）。
"""

from src.follow.follow_controller import (
    FollowController, FollowParams,
    L_SHOULDER, R_SHOULDER, NUM_LANDMARKS,
)

W, H = 640, 480
PARAMS = FollowParams(target_sw=0.19)  # 目標肩幅 0.19 ≒ 122px


def make_lm(center_px=320, sw_px=122, y=240):
    """人が center_px に立ち肩幅 sw_px で見えるフレーム."""
    lm = [[i, W // 2, H // 2] for i in range(NUM_LANDMARKS)]
    lm[R_SHOULDER] = [R_SHOULDER, center_px - sw_px / 2, y]  # 画面左=人物の右
    lm[L_SHOULDER] = [L_SHOULDER, center_px + sw_px / 2, y]
    return lm


def settle(ctrl, lm, n=30):
    """ローパスが収束するまで同フレームを流し、最終 status を返す."""
    s = None
    for _ in range(n):
        s = ctrl.update(lm, W, H)
    return s


def test_far_person_moves_forward():
    ctrl = FollowController(PARAMS)
    s = settle(ctrl, make_lm(sw_px=60))   # sw=0.094 < 0.19 → 遠い
    assert s.vx > 0 and s.label == "FOLLOW"


def test_close_person_backs_off():
    ctrl = FollowController(PARAMS)
    s = settle(ctrl, make_lm(sw_px=200))  # sw=0.31 > 0.19 → 近すぎ
    assert s.vx < 0 and s.label == "TOO CLOSE"
    assert s.vx >= -PARAMS.max_back_vx - 1e-9


def test_at_target_distance_holds():
    ctrl = FollowController(PARAMS)
    s = settle(ctrl, make_lm(sw_px=122))  # sw≒target（デッドバンド内）
    assert s.vx == 0.0 and s.omega == 0.0 and s.label == "HOLD"


def test_person_left_turns_left():
    # 人が画面左（= ロボットの左）→ 左旋回 = omega 正
    ctrl = FollowController(PARAMS)
    s = settle(ctrl, make_lm(center_px=160, sw_px=122))
    assert s.omega > 0


def test_person_right_turns_right():
    ctrl = FollowController(PARAMS)
    s = settle(ctrl, make_lm(center_px=480, sw_px=122))
    assert s.omega < 0


def test_centered_no_turn():
    ctrl = FollowController(PARAMS)
    s = settle(ctrl, make_lm(center_px=320, sw_px=122))
    assert s.omega == 0.0


def test_lost_target_stops_immediately():
    ctrl = FollowController(PARAMS)
    settle(ctrl, make_lm(sw_px=60))       # 前進中
    s = ctrl.update([], W, H)             # 見失い → 1 フレームで停止
    assert s.vx == 0.0 and s.omega == 0.0 and s.label == "NO TARGET"


def test_reacquire_does_not_carry_old_command():
    ctrl = FollowController(PARAMS)
    settle(ctrl, make_lm(sw_px=60))       # 前進で収束
    ctrl.update([], W, H)                 # 見失い（リセット）
    s = ctrl.update(make_lm(sw_px=122), W, H)  # 目標距離で再検出
    assert abs(s.vx) < 0.1                # 古い前進指令を引きずらない


def test_output_clamped():
    ctrl = FollowController(PARAMS)
    s = settle(ctrl, make_lm(center_px=630, sw_px=20))  # 極端に遠く端
    assert s.vx <= PARAMS.max_vx + 1e-9
    assert abs(s.omega) <= PARAMS.max_omega + 1e-9
