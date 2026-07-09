"""対象人物の選択・追跡（純ロジック・ROS/TensorRT 非依存）.

YOLO は複数人を検出するため（BlazePose は単人前提だった）、
「どの人を追従するか」を決めてフレーム間で同一人物を保持する。

方針（安全側・シンプル）:
- 未ロック時: 最も大きい bbox（= 最も近い人）をロックする
- ロック中: 前フレームの bbox と IoU が最大の検出を同一人物とみなす。
  IoU が min_iou 未満しかない場合は「その人が見えない」フレームとして扱う
- lost_grace_sec を超えて見えなければロック解除（次に見えた最近接へ）
- **勝手に別人へ乗り移らない**（IoU 連続性が唯一の同一性根拠）
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class TrackerParams:
    min_iou: float = 0.3        # 同一人物とみなす IoU 下限
    lost_grace_sec: float = 1.0  # ロック保持時間（超えたら解除）


def _iou(a, b):
    """bbox (x1,y1,x2,y2) の IoU."""
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def _area(b):
    return (b[2] - b[0]) * (b[3] - b[1])


class PersonTracker:
    """複数検出から追従対象 1 人を選択・保持する状態機械."""

    def __init__(self, params: TrackerParams = TrackerParams()):
        self._p = params
        self._bbox = None       # ロック中の対象 bbox
        self._last_seen = None

    def reset(self) -> None:
        self._bbox = None
        self._last_seen = None

    @property
    def locked(self) -> bool:
        return self._bbox is not None

    def update(self, detections, now: float):
        """検出リストから追従対象を返す.

        Args:
            detections: [(bbox(x1,y1,x2,y2), 任意のペイロード)] のリスト。
            now: 現在時刻 [秒]。

        Returns:
            選択された (bbox, payload) 。対象なしなら None。
        """
        # ロック期限切れの解除
        if (self._bbox is not None and self._last_seen is not None and
                now - self._last_seen > self._p.lost_grace_sec):
            self.reset()

        if not detections:
            return None

        if self._bbox is None:
            # 新規ロック: 最大 bbox（最近接）
            best = max(detections, key=lambda d: _area(d[0]))
            self._bbox = best[0]
            self._last_seen = now
            return best

        # ロック継続: IoU 最大の検出に追随
        best = max(detections, key=lambda d: _iou(self._bbox, d[0]))
        if _iou(self._bbox, best[0]) < self._p.min_iou:
            return None   # 対象が見えないフレーム（grace 中はロック維持）
        self._bbox = best[0]
        self._last_seen = now
        return best
