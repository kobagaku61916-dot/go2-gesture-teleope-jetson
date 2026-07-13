"""見失い時の探索旋回ステートマシン（純ロジック・ROS 非依存）.

follow_controller v2（速度計算）はそのままに、その外側で
TRACKING / SEARCHING / STOP の 3 状態を管理するオーケストレータ。

状態遷移:
    TRACKING  : 対象を追従中（検出中 + 見失い猶予 COAST を含む）。
                毎フレーム、肩中点のピクセル X を監視し「画面のどの帯で
                最後に見えたか」（LEFT / CENTER / RIGHT）を記憶する
    SEARCHING : 猶予超過で見失い、かつ最後の位置が左右エッジ帯だった場合。
                消えた方向へ**その場旋回のみ**（linear.x = 0、angular.z = ±search_omega）。
                再発見で TRACKING へ復帰
    STOP      : ①探索が search_timeout_sec 続いても再発見できない
                ②画面中央で消えた（方向不明。安全のため探索しない）
                → 速度完全ゼロ。対象が再び見えたら TRACKING へ復帰

安全設計:
- 探索中は前進しない（旋回のみ）。旋回速度は safety_gate クランプ(0.8)より低い既定 0.5
- タイムアウトで必ず停止（回り続けない）
- 方向が分からない消え方では探索しない（中央ロスト = 遮蔽の可能性 → その場停止）
- 後段の safety_gate（クランプ + watchdog）は無改造のまま最終防壁として機能
"""

from dataclasses import dataclass
from enum import Enum

from .follow_controller import FollowController, FollowParams, L_SHOULDER, R_SHOULDER

NUM_LANDMARKS = 33


class FollowState(Enum):
    TRACKING = "TRACKING"
    SEARCHING = "SEARCHING"
    STOP = "STOP"


class ExitSide(Enum):
    LEFT = "LEFT"      # 画面左端で最後に観測（= ロボットから見て左）
    CENTER = "CENTER"
    RIGHT = "RIGHT"


@dataclass(frozen=True)
class SearchParams:
    edge_margin_px: int = 100      # 画面端とみなす帯の幅 [px]
    search_omega: float = 0.8      # 探索旋回速度 [rad/s]（= safety_gate クランプ上限）
    search_timeout_sec: float = 4.0  # 探索の上限時間 → STOP（0.8rad/s×4s ≒ 180°）
    exit_vel_px: float = 4.0       # 離脱方向を速度から推定する下限 [px/フレーム]
                                   # （速い横抜けは最後の位置が中央でも方向が分かる）


@dataclass(frozen=True)
class FollowOutput:
    vx: float
    omega: float
    state: FollowState
    label: str          # 表示・ログ用
    distance_m: float


class FollowStateMachine:
    """追従 + 見失い探索のオーケストレータ.

    速度計算は FollowController に委譲し、本クラスは状態遷移と
    探索指令の生成のみを担う（疎結合）。
    """

    def __init__(self, follow_params: FollowParams = FollowParams(),
                 search_params: SearchParams = SearchParams()):
        self._ctrl = FollowController(follow_params)
        self._p = search_params
        self._state = FollowState.TRACKING
        self._last_side = ExitSide.CENTER
        self._search_start = None
        self._last_px = None
        self._px_vel = 0.0   # 肩中点 X の速度 [px/フレーム]（EMA）

    @property
    def state(self) -> FollowState:
        return self._state

    def reset(self) -> None:
        self._ctrl.reset()
        self._state = FollowState.TRACKING
        self._last_side = ExitSide.CENTER
        self._search_start = None
        self._last_px = None
        self._px_vel = 0.0

    # ------------------------------------------------------------------
    def update(self, lm_list, w, h, now: float) -> FollowOutput:
        detected = len(lm_list) >= NUM_LANDMARKS
        if detected:
            self._remember_side(lm_list, w)
            self._state = FollowState.TRACKING
            self._search_start = None
            fs = self._ctrl.update(lm_list, w, h, now)
            return FollowOutput(fs.vx, fs.omega, self._state, fs.label, fs.distance_m)

        # --- 未検出フレーム ---
        fs = self._ctrl.update(lm_list, w, h, now)
        if fs.label == "COAST":
            # follow_controller の見失い猶予内: 直前指令を維持（状態は TRACKING のまま）
            return FollowOutput(fs.vx, fs.omega, FollowState.TRACKING, "COAST", 0.0)

        # 猶予超過（NO TARGET）
        if self._state == FollowState.TRACKING:
            side = self._infer_exit_side()
            if side in (ExitSide.LEFT, ExitSide.RIGHT):
                self._last_side = side
                self._state = FollowState.SEARCHING
                self._search_start = now
            else:
                # 位置も速度も方向を示さない（静止したまま遮蔽など）→ 探索せず停止
                self._state = FollowState.STOP

        if self._state == FollowState.SEARCHING:
            if now - self._search_start >= self._p.search_timeout_sec:
                self._state = FollowState.STOP
            else:
                # 消えた方向へその場旋回。画面左 = ロボット左 = angular.z 正
                omega = (self._p.search_omega if self._last_side == ExitSide.LEFT
                         else -self._p.search_omega)
                return FollowOutput(0.0, omega, self._state,
                                    f"SEARCH-{self._last_side.value}", 0.0)

        return FollowOutput(0.0, 0.0, FollowState.STOP, "STOP(LOST)", 0.0)

    # ------------------------------------------------------------------
    def _remember_side(self, lm_list, w) -> None:
        """可視フレームごとに肩中点 X の帯と横方向速度を記憶する."""
        px = (lm_list[L_SHOULDER][1] + lm_list[R_SHOULDER][1]) / 2.0
        if self._last_px is not None:
            # EMA で平滑化した横速度 [px/フレーム]（離脱方向の推定に使う）
            self._px_vel = 0.7 * self._px_vel + 0.3 * (px - self._last_px)
        self._last_px = px
        if px < self._p.edge_margin_px:
            self._last_side = ExitSide.LEFT
        elif px > w - self._p.edge_margin_px:
            self._last_side = ExitSide.RIGHT
        else:
            self._last_side = ExitSide.CENTER

    def _infer_exit_side(self) -> ExitSide:
        """離脱方向を推定する: ①端の帯で消えた ②速い横移動中に消えた の順で判定."""
        if self._last_side in (ExitSide.LEFT, ExitSide.RIGHT):
            return self._last_side
        if abs(self._px_vel) >= self._p.exit_vel_px:
            return ExitSide.RIGHT if self._px_vel > 0 else ExitSide.LEFT
        return ExitSide.CENTER
