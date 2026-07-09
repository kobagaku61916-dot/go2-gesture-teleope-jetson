"""coco_adapter / person_tracker の単体テスト（rclpy / tensorrt 不要）."""

from src.detect.coco_adapter import coco_to_lm_list, NUM_LANDMARKS
from src.detect.person_tracker import PersonTracker, TrackerParams
from src.gesture.gesture_mapper import compute_command, GestureParams
from src.pose.confidence import key_landmarks_visible

W, H = 640, 480


# --- coco_adapter -------------------------------------------------------

def make_kpts(ls=(240, 240), rs=(400, 240), lw=(230, 400), rw=(410, 400), conf=0.9):
    """COCO 17 点（使う 4 点だけ指定・他は原点）."""
    kpts = [(0.0, 0.0, 0.0)] * 17
    kpts[5] = (*ls, conf)   # left_shoulder → MP11
    kpts[6] = (*rs, conf)   # right_shoulder → MP12
    kpts[9] = (*lw, conf)   # left_wrist → MP15
    kpts[10] = (*rw, conf)  # right_wrist → MP16
    return kpts


def test_adapter_maps_key_points():
    lm, vis = coco_to_lm_list(make_kpts())
    assert len(lm) == NUM_LANDMARKS and len(vis) == NUM_LANDMARKS
    assert lm[11][1:] == [240, 240] and lm[12][1:] == [400, 240]
    assert lm[15][1:] == [230, 400] and lm[16][1:] == [410, 400]
    assert vis[11] == vis[16] == 0.9 and vis[0] == 0.0


def test_adapter_output_works_with_gesture_mapper():
    # 旧テストと同じ「手を下ろした STOP 姿勢」が STOP になる（下流互換の保証）
    # 注: MP11=人物の左肩は画面右側に写る（COCO left_shoulder も同じ解剖学的左）
    lm, _ = coco_to_lm_list(make_kpts(ls=(400, 240), rs=(240, 240),
                                      lw=(410, 400), rw=(230, 400)))
    vx, vy, omega, label = compute_command(lm, W, H, GestureParams())
    assert label == "STOP" and vx == 0.0


def test_adapter_output_works_with_confidence_gate():
    lm, vis = coco_to_lm_list(make_kpts(conf=0.9))
    assert key_landmarks_visible(vis, 0.3)               # 4 点すべて
    assert key_landmarks_visible(vis, 0.3, key_ids=(11, 12))  # 肩のみ（follow）
    lm, vis = coco_to_lm_list(make_kpts(conf=0.1))
    assert not key_landmarks_visible(vis, 0.3)


def test_adapter_short_input_returns_empty():
    lm, vis = coco_to_lm_list([(0, 0, 0)] * 5)
    assert lm == [] and vis == []


# --- person_tracker -----------------------------------------------------

def bbox(cx, cy, w=100, h=200):
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def test_locks_largest_person():
    tr = PersonTracker()
    near = (bbox(320, 240, 200, 400), "near")
    far = (bbox(500, 240, 60, 120), "far")
    got = tr.update([far, near], now=0.0)
    assert got[1] == "near" and tr.locked


def test_keeps_lock_by_iou_not_size():
    tr = PersonTracker()
    tr.update([(bbox(320, 240, 100, 200), "A")], now=0.0)
    # 大きい別人 B が現れても、動きの連続する A を追い続ける
    a2 = (bbox(330, 240, 100, 200), "A")
    b = (bbox(600, 300, 300, 400), "B")
    got = tr.update([b, a2], now=0.1)
    assert got[1] == "A"


def test_target_hidden_returns_none_but_keeps_lock():
    tr = PersonTracker(TrackerParams(lost_grace_sec=1.0))
    tr.update([(bbox(320, 240), "A")], now=0.0)
    # A が消えて遠くに別人 B だけ → 乗り移らず None
    got = tr.update([(bbox(600, 100, 50, 100), "B")], now=0.2)
    assert got is None and tr.locked


def test_lock_released_after_grace():
    tr = PersonTracker(TrackerParams(lost_grace_sec=1.0))
    tr.update([(bbox(320, 240), "A")], now=0.0)
    got = tr.update([(bbox(600, 100), "B")], now=1.5)  # 1.0s 超過 → 解除・再ロック
    assert got[1] == "B"


def test_no_detections_returns_none():
    tr = PersonTracker()
    assert tr.update([], now=0.0) is None
