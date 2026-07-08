"""gesture_mapper.compute_command の単体テスト（rclpy 不要）.

旧リポ gesture_ope.computeCommand との等価性を、代表的な姿勢パターンで検証する。
座標系: 画面左上原点・左右反転なし（画面左側＝人物の右）。
"""

import pytest

from src.gesture.gesture_mapper import (
    GestureParams,
    L_SHOULDER, R_SHOULDER, L_WRIST, R_WRIST,
    NUM_LANDMARKS,
    compute_command,
)

# テスト用フレームサイズ
W, H = 640, 480

# 基準姿勢（ピクセル）: 画面中央に立つ人物。
# 画面左側＝人物の右なので、右肩の x < 左肩の x。肩幅 160px = sw 0.25。
R_SHOULDER_XY = (240, 240)
L_SHOULDER_XY = (400, 240)
# 手を下ろした位置（肩より下）
R_WRIST_DOWN = (230, 400)
L_WRIST_DOWN = (410, 400)


def make_lm_list(r_wrist=R_WRIST_DOWN, l_wrist=L_WRIST_DOWN,
                 r_shoulder=R_SHOULDER_XY, l_shoulder=L_SHOULDER_XY):
    """33 点の lmList（[id,x,y]）を合成する。指定外の点は画面中央."""
    lm = [[i, W // 2, H // 2] for i in range(NUM_LANDMARKS)]
    lm[R_SHOULDER] = [R_SHOULDER, *r_shoulder]
    lm[L_SHOULDER] = [L_SHOULDER, *l_shoulder]
    lm[R_WRIST] = [R_WRIST, *r_wrist]
    lm[L_WRIST] = [L_WRIST, *l_wrist]
    return lm


P = GestureParams()


def test_no_body_when_landmarks_missing():
    assert compute_command([], W, H) == (0.0, 0.0, 0.0, "NO BODY")
    assert compute_command(make_lm_list()[:32], W, H) == (0.0, 0.0, 0.0, "NO BODY")


def test_stop_when_both_hands_down():
    vx, vy, omega, label = compute_command(make_lm_list(), W, H)
    assert (vx, vy, omega, label) == (0.0, 0.0, 0.0, "STOP")


def test_forward_right_hand_raised():
    # 右手首を右肩の真上（十分上）に。sw=0.25 → raise_margin*sw = 0.125 = 60px
    lm = make_lm_list(r_wrist=(240, 100))
    vx, vy, omega, label = compute_command(lm, W, H)
    assert vx == pytest.approx(P.linear_speed)
    assert omega == 0.0
    assert label == "FORWARD"


def test_backward_left_hand_raised():
    lm = make_lm_list(l_wrist=(400, 100))
    vx, vy, omega, label = compute_command(lm, W, H)
    assert vx == pytest.approx(-P.linear_speed)
    assert omega == 0.0
    assert label == "BACKWARD"


def test_turn_right_right_hand_extended():
    # 右手首を肩の高さで画面左（x 小）へ。extend_margin*sw = 0.25 = 160px 以上離す
    lm = make_lm_list(r_wrist=(40, 240))
    vx, vy, omega, label = compute_command(lm, W, H)
    assert vx == 0.0
    assert omega == pytest.approx(-P.angular_speed)
    assert label == "TURN-RIGHT"


def test_turn_left_left_hand_extended():
    lm = make_lm_list(l_wrist=(600, 240))
    vx, vy, omega, label = compute_command(lm, W, H)
    assert vx == 0.0
    assert omega == pytest.approx(P.angular_speed)
    assert label == "TURN-LEFT"


def test_forward_and_turn_left_combined():
    # 右手上げ（前進）+ 左手横伸ばし（左旋回）は組み合わせ可能
    lm = make_lm_list(r_wrist=(240, 100), l_wrist=(600, 240))
    vx, vy, omega, label = compute_command(lm, W, H)
    assert vx == pytest.approx(P.linear_speed)
    assert omega == pytest.approx(P.angular_speed)
    assert label == "FORWARD + TURN-LEFT"


def test_both_hands_raised_cancels_vx():
    # 両手上げは前進・後進が打ち消し合い vx=0
    lm = make_lm_list(r_wrist=(240, 100), l_wrist=(400, 100))
    vx, vy, omega, label = compute_command(lm, W, H)
    assert vx == 0.0
    assert label == "STOP"


def test_raised_but_offset_hand_not_forward():
    # 上げていても横ズレが raise_near*sw = 0.3 = 192px を超えると「上げ」と見なさない
    lm = make_lm_list(r_wrist=(40, 100))
    vx, vy, omega, label = compute_command(lm, W, H)
    assert vx == 0.0


def test_extended_but_not_level_not_turn():
    # 横に伸ばしても高さズレが level_margin*sw = 0.15 = 72px を超えると旋回しない
    lm = make_lm_list(r_wrist=(40, 340))
    vx, vy, omega, label = compute_command(lm, W, H)
    assert omega == 0.0


def test_custom_params_change_speeds():
    p = GestureParams(linear_speed=0.3, angular_speed=0.8)
    lm = make_lm_list(r_wrist=(240, 100))
    vx, _, _, _ = compute_command(lm, W, H, p)
    assert vx == pytest.approx(0.3)


def test_vy_always_zero():
    # 現仕様では vy は常に 0（将来拡張枠）
    for lm in (make_lm_list(), make_lm_list(r_wrist=(240, 100))):
        _, vy, _, _ = compute_command(lm, W, H)
        assert vy == 0.0
