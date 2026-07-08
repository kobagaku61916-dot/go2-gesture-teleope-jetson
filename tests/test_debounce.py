"""debounce.CommandDebouncer の単体テスト（rclpy 不要）."""

import pytest

from src.gesture.debounce import CommandDebouncer, ZERO

FWD = (0.2, 0.0, 0.0)
TURN = (0.0, 0.0, 0.3)


def test_nonzero_requires_n_consecutive_frames():
    d = CommandDebouncer(debounce_frames=3)
    assert d.update(FWD) == ZERO
    assert d.update(FWD) == ZERO
    assert d.update(FWD) == FWD      # 3 フレーム目で通る
    assert d.update(FWD) == FWD      # 以降は通り続ける


def test_stop_passes_immediately():
    d = CommandDebouncer(debounce_frames=3)
    d.update(FWD); d.update(FWD); d.update(FWD)
    assert d.update(ZERO) == ZERO    # STOP は即時


def test_single_frame_glitch_is_suppressed():
    d = CommandDebouncer(debounce_frames=3)
    # 1 フレームだけの誤検出（旧リポで実測したパターン）は出力されない
    assert d.update(FWD) == ZERO
    assert d.update(ZERO) == ZERO
    assert d.update(ZERO) == ZERO


def test_command_change_resets_count():
    d = CommandDebouncer(debounce_frames=3)
    d.update(FWD); d.update(FWD)
    assert d.update(TURN) == ZERO    # 別指令に変わったらカウントし直し
    d.update(TURN)
    assert d.update(TURN) == TURN


def test_stop_resets_count():
    d = CommandDebouncer(debounce_frames=3)
    d.update(FWD); d.update(FWD)
    d.update(ZERO)
    assert d.update(FWD) == ZERO     # STOP を挟んだら 1 からやり直し


def test_frames_one_passes_through():
    d = CommandDebouncer(debounce_frames=1)
    assert d.update(FWD) == FWD


def test_invalid_frames_raises():
    with pytest.raises(ValueError):
        CommandDebouncer(debounce_frames=0)
