"""wave_detector.WaveDetector の単体テスト（rclpy 不要）.

座標系・基準姿勢は test_gesture_mapper.py と同じ（画面左上原点・肩幅 160px = sw 0.25）。
"""

import math

from src.gesture.wave_detector import (
    WaveDetector, WaveParams,
    L_SHOULDER, R_SHOULDER, R_WRIST, NUM_LANDMARKS,
)

W, H = 640, 480
FPS = 30.0
DT = 1.0 / FPS

R_SHOULDER_XY = (240, 240)
L_SHOULDER_XY = (400, 240)
SW_PX = 160  # 肩幅ピクセル

PARAMS = WaveParams(min_amplitude=0.25, min_swings=4,
                    min_duration_sec=2.0, max_gap_sec=0.8, cooldown_sec=10.0)


def make_lm(r_wrist):
    lm = [[i, W // 2, H // 2] for i in range(NUM_LANDMARKS)]
    lm[R_SHOULDER] = [R_SHOULDER, *R_SHOULDER_XY]
    lm[L_SHOULDER] = [L_SHOULDER, *L_SHOULDER_XY]
    lm[R_WRIST] = [R_WRIST, *r_wrist]
    return lm


def wave_frame(t, freq_hz=1.5, amp_px=80, y=140):
    """時刻 t の「右手を上げて左右に振っている」フレーム（amp 80px = 0.5sw）."""
    x = R_SHOULDER_XY[0] + amp_px * math.sin(2 * math.pi * freq_hz * t)
    return make_lm((x, y))


def run_wave(det, t0, duration, **kw):
    results = []
    t = t0
    for _ in range(int(duration * FPS)):
        results.append(det.update(wave_frame(t, **kw), W, H, t))
        t += DT
    return results, t


def run_static(det, t0, duration, r_wrist):
    results = []
    t = t0
    lm = make_lm(r_wrist)
    for _ in range(int(duration * FPS)):
        results.append(det.update(lm, W, H, t))
        t += DT
    return results, t


def test_wave_2s_triggers_once():
    det = WaveDetector(PARAMS)
    results, _ = run_wave(det, 0.0, 3.5)
    assert sum(s.triggered for s in results) == 1


def test_static_raise_never_triggers():
    # 右手を上げて静止（FORWARD 姿勢）→ 振り検出も抑制も発生しない
    det = WaveDetector(PARAMS)
    results, _ = run_static(det, 0.0, 4.0, (245, 100))
    assert not any(s.triggered or s.waving for s in results)


def test_wave_below_shoulder_never_triggers():
    # 肩より下で振っても発火しない
    det = WaveDetector(PARAMS)
    results, _ = run_wave(det, 0.0, 3.5, y=400)
    assert not any(s.triggered or s.waving for s in results)


def test_short_wave_does_not_trigger():
    det = WaveDetector(PARAMS)
    results, t = run_wave(det, 0.0, 1.0)   # 1 秒 < 2 秒
    r2, _ = run_static(det, t, 2.0, (230, 400))  # 手を下ろす
    assert not any(s.triggered for s in results + r2)


def test_waving_sets_active_for_suppression():
    det = WaveDetector(PARAMS)
    results, _ = run_wave(det, 0.0, 1.0)
    # 最初の方向転換以降は waving（テレオペ抑制）が立つ
    assert any(s.waving for s in results)


def test_cooldown_prevents_retrigger():
    det = WaveDetector(PARAMS)
    results, _ = run_wave(det, 0.0, 8.0)   # 振り続けても cooldown 内は 1 回
    assert sum(s.triggered for s in results) == 1


def test_no_body_resets():
    det = WaveDetector(PARAMS)
    _, t = run_wave(det, 0.0, 1.5)
    det.update([], W, H, t)  # NO BODY
    results, _ = run_wave(det, t, 1.0)  # 再開 1 秒では発火しない
    assert not any(s.triggered for s in results)
