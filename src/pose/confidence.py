"""姿勢推定の信頼度チェック（純ロジック・ROS/MediaPipe 非依存）.

Go2 搭載カメラは振動・照明変化が大きく、低信頼度の「なんとなく人に見える」
検出でジェスチャー判定するのは危険（architecture.md §4）。
判定に使う主要ランドマーク（両肩・両手首）の visibility が下限未満なら
そのフレームは NO BODY（0 指令）として扱う。
"""

# gesture_mapper が判定に使うランドマーク（両肩・両手首）
KEY_LANDMARK_IDS = (11, 12, 15, 16)


def key_landmarks_visible(visibilities, min_visibility: float,
                          key_ids=KEY_LANDMARK_IDS) -> bool:
    """主要ランドマークすべての visibility が下限以上か.

    Args:
        visibilities: ランドマーク ID を添字とする visibility のシーケンス
            （PoseTracker.find_visibilities() の戻り値。未検出なら空）。
        min_visibility: 下限（0.0〜1.0）。0.0 ならチェック無効。
        key_ids: 判定対象のランドマーク ID。

    Returns:
        すべて下限以上なら True。ランドマーク不足なら False。
    """
    if min_visibility <= 0.0:
        return True
    if len(visibilities) <= max(key_ids):
        return False
    return all(visibilities[i] >= min_visibility for i in key_ids)
