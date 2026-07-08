"""greet_detector.GreetDetector の単体テスト（rclpy 不要）.

座標系・基準姿勢は test_gesture_mapper.py と同じ（画面左上原点・肩幅 160px）。
"""

from src.gesture.greet_detector import (
    GreetDetector, GreetParams,
    L_SHOULDER, R_SHOULDER, L_WRIST, R_WRIST, NUM_LANDMARKS,
)

W, H = 640, 480
FPS = 30.0
DT = 1.0 / FPS

R_SHOULDER_XY = (240, 240)
L_SHOULDER_XY = (400, 240)
R_WRIST_DOWN = (230, 400)
L_WRIST_DOWN = (410, 400)
R_WRIST_UP = (245, 100)     # 右肩の真上（かざし）
L_WRIST_UP = (395, 100)     # 左肩の真上

PARAMS = GreetParams(hold_sec=1.5, cooldown_sec=10.0)


def make_lm(r_wrist=R_WRIST_DOWN, l_wrist=L_WRIST_DOWN):
    lm = [[i, W // 2, H // 2] for i in range(NUM_LANDMARKS)]
    lm[R_SHOULDER] = [R_SHOULDER, *R_SHOULDER_XY]
    lm[L_SHOULDER] = [L_SHOULDER, *L_SHOULDER_XY]
    lm[R_WRIST] = [R_WRIST, *r_wrist]
    lm[L_WRIST] = [L_WRIST, *l_wrist]
    return lm


BOTH_UP = make_lm(r_wrist=R_WRIST_UP, l_wrist=L_WRIST_UP)
ONLY_RIGHT = make_lm(r_wrist=R_WRIST_UP)
DOWN = make_lm()


def run(det, lm, t0, duration):
    results = []
    t = t0
    for _ in range(int(duration * FPS)):
        results.append(det.update(lm, W, H, t))
        t += DT
    return results, t


def test_hold_triggers_once():
    det = GreetDetector(PARAMS)
    results, _ = run(det, BOTH_UP, 0.0, 2.5)
    assert sum(s.triggered for s in results) == 1
    assert results[0].holding and not results[0].triggered  # 即発火はしない


def test_short_hold_does_not_trigger():
    det = GreetDetector(PARAMS)
    results, t = run(det, BOTH_UP, 0.0, 1.0)   # 1.0s < hold 1.5s
    r2, _ = run(det, DOWN, t, 1.0)
    assert not any(s.triggered for s in results + r2)


def test_one_hand_is_not_greet():
    det = GreetDetector(PARAMS)
    results, _ = run(det, ONLY_RIGHT, 0.0, 3.0)  # FORWARD 姿勢
    assert not any(s.holding or s.triggered for s in results)


def test_cooldown_prevents_retrigger():
    det = GreetDetector(PARAMS)
    results, _ = run(det, BOTH_UP, 0.0, 8.0)   # 持ち続けても cooldown 内は 1 回
    assert sum(s.triggered for s in results) == 1


def test_retrigger_after_cooldown():
    det = GreetDetector(PARAMS)
    r1, t = run(det, BOTH_UP, 0.0, 2.0)
    _, t = run(det, DOWN, t, PARAMS.cooldown_sec)
    r2, _ = run(det, BOTH_UP, t, 2.0)
    assert sum(s.triggered for s in r1) == 1
    assert sum(s.triggered for s in r2) == 1


def test_no_body_resets_hold():
    det = GreetDetector(PARAMS)
    _, t = run(det, BOTH_UP, 0.0, 1.2)
    det.update([], W, H, t)  # NO BODY
    results, _ = run(det, BOTH_UP, t, 1.0)  # 再開 1.0s では発火しない
    assert not any(s.triggered for s in results)
