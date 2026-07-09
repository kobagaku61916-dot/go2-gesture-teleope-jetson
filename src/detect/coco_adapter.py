"""COCO 17 キーポイント → MediaPipe 33 ランドマーク形式への写像（純ロジック）.

下流（gesture_mapper / dance / wave / follow_controller / confidence）はすべて
MediaPipe の lm_list（[id, x, y] × 33、ピクセル座標）と visibility リストを
前提に実装・実機検証済みのため、YOLO-pose の出力をこの形式に合わせる
アダプタを置くことで**下流を無改造で再利用**する。

対応表（実際に使うのは肩・手首の 4 点のみ）:
    MediaPipe 11 (L_SHOULDER) ← COCO 5 (left_shoulder)
    MediaPipe 12 (R_SHOULDER) ← COCO 6 (right_shoulder)
    MediaPipe 15 (L_WRIST)    ← COCO 9 (left_wrist)
    MediaPipe 16 (R_WRIST)    ← COCO 10 (right_wrist)

未使用の id は両肩中点で埋め、visibility は 0.0 とする（confidence.py の
key_ids 検査は使う点しか見ないため無害。0.0 なら誤って他判定に使われても
NO BODY 側に倒れる）。
"""

NUM_LANDMARKS = 33
_COCO_TO_MP = {5: 11, 6: 12, 9: 15, 10: 16}


def coco_to_lm_list(kpts):
    """COCO キーポイント配列を MediaPipe 形式 lm_list と visibility に変換する.

    Args:
        kpts: [(x, y, conf)] × 17（ピクセル座標。YOLO-pose の 1 人分）。

    Returns:
        (lm_list, visibilities):
            lm_list: [id, x, y] × 33
            visibilities: 長さ 33 の conf リスト（未対応点は 0.0）
    """
    if len(kpts) < 17:
        return [], []
    ls, rs = kpts[5], kpts[6]
    cx, cy = (ls[0] + rs[0]) / 2.0, (ls[1] + rs[1]) / 2.0
    lm_list = [[i, cx, cy] for i in range(NUM_LANDMARKS)]
    vis = [0.0] * NUM_LANDMARKS
    for coco_id, mp_id in _COCO_TO_MP.items():
        x, y, c = kpts[coco_id]
        lm_list[mp_id] = [mp_id, x, y]
        vis[mp_id] = float(c)
    return lm_list, vis
