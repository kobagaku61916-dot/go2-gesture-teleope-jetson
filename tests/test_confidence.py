"""confidence.key_landmarks_visible の単体テスト（rclpy/mediapipe 不要）."""

from src.pose.confidence import KEY_LANDMARK_IDS, key_landmarks_visible

N = 33


def vis(default=0.9, **overrides):
    """33 点の visibility リストを合成する。overrides={id: value}."""
    v = [default] * N
    for i, val in overrides.items():
        v[int(i)] = val
    return v


def test_all_visible_passes():
    assert key_landmarks_visible(vis(), 0.5)


def test_one_key_landmark_below_threshold_fails():
    for key in KEY_LANDMARK_IDS:
        assert not key_landmarks_visible(vis(**{str(key): 0.3}), 0.5)


def test_non_key_landmark_low_visibility_is_ignored():
    # 足（27 等）の visibility が低くても腕ジェスチャー判定は成立する
    assert key_landmarks_visible(vis(**{"27": 0.0, "28": 0.0}), 0.5)


def test_empty_list_fails():
    assert not key_landmarks_visible([], 0.5)


def test_threshold_zero_disables_check():
    assert key_landmarks_visible([], 0.0)
    assert key_landmarks_visible(vis(default=0.0), 0.0)


def test_boundary_value_passes():
    assert key_landmarks_visible(vis(**{"11": 0.5}), 0.5)
