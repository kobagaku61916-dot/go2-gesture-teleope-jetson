"""dance_detector.DanceDetector の単体テスト（rclpy 不要）.

「右手と左手の交互伸び縮み」パターンの発火条件・不応期（cooldown）・
パターン中断（間隔超過）・NO BODY リセット・テレオペ抑制（is_active）を
合成ランドマーク時系列で検証する。
座標系・基準姿勢は test_gesture_mapper.py と同じ（画面左側＝人物の右）。
"""

from src.gesture.dance_detector import (
    DanceDetector,
    DanceParams,
    L_SHOULDER, R_SHOULDER, L_WRIST, R_WRIST,
    NUM_LANDMARKS,
)

W, H = 640, 480
FPS = 30.0
DT = 1.0 / FPS

# 画面中央に立つ人物。肩幅 160px = sw 0.25。
R_SHOULDER_XY = (240, 240)
L_SHOULDER_XY = (400, 240)
R_WRIST_DOWN = (230, 400)     # 手を下ろした位置（横オフセットほぼなし）
L_WRIST_DOWN = (410, 400)
R_WRIST_EXT = (40, 240)       # 右手を右方向（画面左）へ 200px = 1.25sw 伸ばした
L_WRIST_EXT = (600, 240)      # 左手を左方向（画面右）へ 200px = 1.25sw 伸ばした

PARAMS = DanceParams(extend_ratio=1.0, retract_ratio=0.6,
                     min_duration_sec=5.0, min_swaps=4,
                     max_interval_sec=2.0, cooldown_sec=15.0)


def make_lm_list(r_wrist=R_WRIST_DOWN, l_wrist=L_WRIST_DOWN):
    """33 点の lmList（[id,x,y]）を合成する。指定外の点は画面中央."""
    lm = [[i, W // 2, H // 2] for i in range(NUM_LANDMARKS)]
    lm[R_SHOULDER] = [R_SHOULDER, *R_SHOULDER_XY]
    lm[L_SHOULDER] = [L_SHOULDER, *L_SHOULDER_XY]
    lm[R_WRIST] = [R_WRIST, *r_wrist]
    lm[L_WRIST] = [L_WRIST, *l_wrist]
    return lm


def pose_frame(side):
    """side: 'R'=右手だけ伸ばす / 'L'=左手だけ伸ばす / None=両手下ろす."""
    if side == "R":
        return make_lm_list(r_wrist=R_WRIST_EXT)
    if side == "L":
        return make_lm_list(l_wrist=L_WRIST_EXT)
    return make_lm_list()


def run_sequence(det, t0, poses, hold_sec):
    """poses の各姿勢を hold_sec ずつ保持して流し、DanceStatus のリストを返す."""
    results = []
    t = t0
    n = int(hold_sec * FPS)
    for side in poses:
        lm = pose_frame(side)
        for _ in range(n):
            results.append(det.update(lm, W, H, t))
            t += DT
    return results, t


def alternating_poses(n_swaps):
    """R→L→R→… と n_swaps 回切り替わる姿勢列（縮める中立を挟む）."""
    poses = []
    side = "R"
    poses.append(side)
    for _ in range(n_swaps):
        poses.append(None)              # いったん縮める
        side = "L" if side == "R" else "R"
        poses.append(side)
    return poses


def test_alternating_5s_triggers_once():
    # R/L を 0.4s 保持 + 中立 0.2s → 1 スワップ 0.6s。10 スワップ ≈ 6.4s
    det = DanceDetector(PARAMS)
    results = []
    t = 0.0
    for side in alternating_poses(10):
        hold = 0.4 if side else 0.2
        lm = pose_frame(side)
        for _ in range(int(hold * FPS)):
            results.append(det.update(lm, W, H, t))
            t += DT
    assert sum(s.triggered for s in results) == 1


def test_static_and_teleop_poses_never_trigger():
    det = DanceDetector(PARAMS)
    # 手下ろし 3 秒 → 右手伸ばし（旋回ジェスチャー）を 6 秒保持 → 発火しない
    results, t = run_sequence(det, 0.0, [None], 3.0)
    r2, _ = run_sequence(det, t, ["R"], 6.0)
    results += r2
    assert not any(s.triggered for s in results)
    # 片手保持だけでは抑制もかからない（旋回テレオペを邪魔しない）
    assert not any(s.is_active for s in results)


def test_too_few_swaps_does_not_trigger():
    det = DanceDetector(PARAMS)
    # 2 スワップだけして止める（合計 5 秒超でも回数不足）
    results = []
    t = 0.0
    for side in ["R", None, "L", None, "R", None, None, None, None]:
        lm = pose_frame(side)
        for _ in range(int(0.7 * FPS)):
            results.append(det.update(lm, W, H, t))
            t += DT
    assert not any(s.triggered for s in results)


def test_long_gap_breaks_chain():
    det = DanceDetector(PARAMS)
    # 3 スワップ → 3 秒中立（max_interval 2s 超過で中断）→ 3 スワップ → 発火しない
    results = []
    t = 0.0
    seq = (["R", None, "L", None, "R", None, "L"] +
           [None] * 5 +   # 中立 0.6s*5 = 3.0s
           ["R", None, "L", None, "R", None, "L"])
    for side in seq:
        lm = pose_frame(side)
        for _ in range(int(0.6 * FPS)):
            results.append(det.update(lm, W, H, t))
            t += DT
    assert not any(s.triggered for s in results)


def test_cooldown_prevents_immediate_retrigger():
    det = DanceDetector(PARAMS)
    results = []
    t = 0.0
    # 交互を 20 スワップ（約 12 秒）続けても、cooldown(15s) 内は 1 回だけ
    for side in alternating_poses(20):
        hold = 0.4 if side else 0.2
        lm = pose_frame(side)
        for _ in range(int(hold * FPS)):
            results.append(det.update(lm, W, H, t))
            t += DT
    assert sum(s.triggered for s in results) == 1


def test_is_active_during_pattern():
    det = DanceDetector(PARAMS)
    results = []
    t = 0.0
    for side in alternating_poses(4):
        hold = 0.4 if side else 0.2
        lm = pose_frame(side)
        for _ in range(int(hold * FPS)):
            results.append(det.update(lm, W, H, t))
            t += DT
    # 2 回目の伸ばし（最初のスワップ）以降は抑制が立つ
    assert any(s.is_active for s in results)


def test_no_body_resets():
    det = DanceDetector(PARAMS)
    t = 0.0
    # 3 スワップ進めてから NO BODY → 続きから再開しても発火しない
    for side in ["R", None, "L", None, "R", None, "L"]:
        lm = pose_frame(side)
        for _ in range(int(0.5 * FPS)):
            det.update(lm, W, H, t)
            t += DT
    det.update([], W, H, t)  # NO BODY
    results = []
    for side in [None, "R", None, "L"]:  # 再開 2 スワップ分
        lm = pose_frame(side)
        for _ in range(int(0.5 * FPS)):
            results.append(det.update(lm, W, H, t))
            t += DT
    assert not any(s.triggered for s in results)
