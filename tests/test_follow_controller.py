"""follow_controller.FollowController v2 の単体テスト（rclpy 不要）.

距離推定: dist = 1.5m × 0.105 / sw。肩幅ピクセルで距離を模擬する（sw = px/640）。
- sw 0.105 ≒ 67px → 1.5m（目標）
- sw 0.0525 ≒ 34px → 3.0m（遠い）
- sw 0.21  ≒ 134px → 0.75m（近すぎ）
"""

from src.follow.follow_controller import (
    FollowController, FollowParams,
    L_SHOULDER, R_SHOULDER, NUM_LANDMARKS,
)

W, H = 640, 480
DT = 1.0 / 13.0  # 実効レート相当
PARAMS = FollowParams()  # 既定値（target 1.5m / sw_at_target 0.105）


def make_lm(center_px=320, sw_px=67, y=240):
    lm = [[i, W // 2, H // 2] for i in range(NUM_LANDMARKS)]
    lm[R_SHOULDER] = [R_SHOULDER, center_px - sw_px / 2, y]
    lm[L_SHOULDER] = [L_SHOULDER, center_px + sw_px / 2, y]
    return lm


def settle(ctrl, lm, t0=0.0, n=40):
    s, t = None, t0
    for _ in range(n):
        s = ctrl.update(lm, W, H, t)
        t += DT
    return s, t


def test_far_person_full_speed():
    # 3m（誤差 1.5m）→ k_dist 0.8 × 1.5 = 1.2 → 上限 0.6 に飽和（v1 の「遅い」を解消）
    ctrl = FollowController(PARAMS)
    s, _ = settle(ctrl, make_lm(sw_px=34))
    assert s.vx > 0.55 and s.label == "FOLLOW"
    assert 2.8 < s.distance_m < 3.4


def test_slightly_far_gentle_speed():
    # 2.1m（誤差 0.6m）→ 0.48 前後の中間速度
    ctrl = FollowController(PARAMS)
    s, _ = settle(ctrl, make_lm(sw_px=48))
    assert 0.3 < s.vx < 0.6


def test_close_person_backs_off_clamped():
    ctrl = FollowController(PARAMS)
    s, _ = settle(ctrl, make_lm(sw_px=134))  # ≒0.75m
    assert s.label == "TOO CLOSE"
    assert -PARAMS.max_back_vx - 1e-9 <= s.vx < 0


def test_at_target_distance_holds():
    ctrl = FollowController(PARAMS)
    s, _ = settle(ctrl, make_lm(sw_px=67))   # 1.5m（デッドバンド内）
    assert s.vx == 0.0 and s.omega == 0.0 and s.label == "HOLD"


def test_person_left_turns_left():
    ctrl = FollowController(PARAMS)
    s, _ = settle(ctrl, make_lm(center_px=160))
    assert s.omega > 0


def test_person_right_turns_right():
    ctrl = FollowController(PARAMS)
    s, _ = settle(ctrl, make_lm(center_px=480))
    assert s.omega < 0


def test_brief_dropout_coasts():
    # 1〜2 フレームの検出落ちでは直前指令を保持（ストップ&ゴー解消）
    ctrl = FollowController(PARAMS)
    s, t = settle(ctrl, make_lm(sw_px=34))
    vx_before = s.vx
    s = ctrl.update([], W, H, t + DT)
    assert s.label == "COAST" and s.vx == vx_before


def test_long_loss_stops():
    # 猶予（0.25s）を超えたら完全停止
    ctrl = FollowController(PARAMS)
    _, t = settle(ctrl, make_lm(sw_px=34))
    s = ctrl.update([], W, H, t + PARAMS.lost_grace_sec + 0.1)
    assert s.vx == 0.0 and s.label == "NO TARGET"


def test_reacquire_after_loss_does_not_carry_old_command():
    ctrl = FollowController(PARAMS)
    _, t = settle(ctrl, make_lm(sw_px=34))
    ctrl.update([], W, H, t + 1.0)               # 猶予超過 → リセット
    s = ctrl.update(make_lm(sw_px=67), W, H, t + 1.1)  # 目標距離で再検出
    assert abs(s.vx) < 0.1


def test_extreme_sw_distance_clamped():
    # 異常に小さい sw でも距離推定は max_est_distance_m で頭打ち
    ctrl = FollowController(PARAMS)
    s, _ = settle(ctrl, make_lm(sw_px=2))
    assert s.distance_m <= PARAMS.max_est_distance_m
    assert s.vx <= PARAMS.max_vx + 1e-9
