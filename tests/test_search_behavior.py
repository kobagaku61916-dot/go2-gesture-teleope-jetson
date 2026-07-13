"""search_behavior.FollowStateMachine の単体テスト（rclpy 不要）.

シナリオ: 対象が画面端へ移動して消える → 消えた方向へ探索旋回 →
タイムアウトで停止 / 再発見で追従復帰。
座標系は他テストと同じ（画面左上原点・幅 640・肩幅 122px ≒ 目標距離想定）。
"""

from src.follow.follow_controller import FollowParams
from src.follow.search_behavior import (
    FollowStateMachine, FollowState, SearchParams,
    L_SHOULDER, R_SHOULDER, NUM_LANDMARKS,
)

W, H = 640, 480
FPS = 13.0
DT = 1.0 / FPS
FP = FollowParams()            # 既定（grace 0.25s）
SP = SearchParams(edge_margin_px=100, search_omega=0.5, search_timeout_sec=3.0)


def make_lm(center_px=320, sw_px=67):
    lm = [[i, W // 2, H // 2] for i in range(NUM_LANDMARKS)]
    lm[R_SHOULDER] = [R_SHOULDER, center_px - sw_px / 2, 240]
    lm[L_SHOULDER] = [L_SHOULDER, center_px + sw_px / 2, 240]
    return lm


def run(sm, lm, t0, duration):
    out, t = None, t0
    for _ in range(int(duration * FPS)):
        out = sm.update(lm, W, H, t)
        t += DT
    return out, t


def lose_target(sm, t0, duration):
    out, t = None, t0
    for _ in range(int(duration * FPS)):
        out = sm.update([], W, H, t)
        t += DT
    return out, t


def track_then_exit(sm, exit_px, t0=0.0):
    """中央で追従 → exit_px の位置で最後に観測 → 消える、まで進める."""
    _, t = run(sm, make_lm(center_px=320), t0, 1.0)
    _, t = run(sm, make_lm(center_px=exit_px), t, 0.3)   # 端の帯で観測
    return t


def test_exit_left_searches_left():
    sm = FollowStateMachine(FP, SP)
    t = track_then_exit(sm, exit_px=60)                 # 左端(<100)で消える
    out, _ = lose_target(sm, t, FP.lost_grace_sec + 0.5)
    assert out.state == FollowState.SEARCHING
    assert out.omega > 0 and out.vx == 0.0              # 左へその場旋回のみ
    assert out.label == "SEARCH-LEFT"


def test_exit_right_searches_right():
    sm = FollowStateMachine(FP, SP)
    t = track_then_exit(sm, exit_px=590)                # 右端(>540)で消える
    out, _ = lose_target(sm, t, FP.lost_grace_sec + 0.5)
    assert out.state == FollowState.SEARCHING
    assert out.omega < 0 and out.vx == 0.0
    assert out.label == "SEARCH-RIGHT"


def test_center_loss_stops_without_search():
    # 中央で消えた（遮蔽など・方向不明）→ 探索せず STOP
    sm = FollowStateMachine(FP, SP)
    _, t = run(sm, make_lm(center_px=320), 0.0, 1.0)
    out, _ = lose_target(sm, t, FP.lost_grace_sec + 0.5)
    assert out.state == FollowState.STOP
    assert out.vx == 0.0 and out.omega == 0.0


def test_search_timeout_stops():
    sm = FollowStateMachine(FP, SP)
    t = track_then_exit(sm, exit_px=60)
    out, _ = lose_target(sm, t, FP.lost_grace_sec + SP.search_timeout_sec + 0.5)
    assert out.state == FollowState.STOP
    assert out.vx == 0.0 and out.omega == 0.0


def test_reacquire_during_search_resumes_tracking():
    sm = FollowStateMachine(FP, SP)
    t = track_then_exit(sm, exit_px=60)
    _, t = lose_target(sm, t, FP.lost_grace_sec + 1.0)   # 探索中
    out, _ = run(sm, make_lm(center_px=320, sw_px=34), t, 1.0)  # 遠くで再発見
    assert out.state == FollowState.TRACKING
    assert out.vx > 0                                    # 追従（前進）再開


def test_reacquire_after_stop_resumes_tracking():
    sm = FollowStateMachine(FP, SP)
    t = track_then_exit(sm, exit_px=60)
    _, t = lose_target(sm, t, FP.lost_grace_sec + SP.search_timeout_sec + 1.0)  # STOP まで
    out, _ = run(sm, make_lm(center_px=320), t, 1.0)
    assert out.state == FollowState.TRACKING


def test_coast_grace_does_not_trigger_search():
    # 猶予(0.25s)以内の瞬断では SEARCHING に入らない（COAST のまま）
    sm = FollowStateMachine(FP, SP)
    t = track_then_exit(sm, exit_px=60)
    out = sm.update([], W, H, t + 0.1)
    assert out.state == FollowState.TRACKING and out.label == "COAST"
