"""手振り（Wave → Greet）検出 — 「右手を上げて左右に振る」（純ロジック・ROS 非依存）.

右手首が右肩より上にある状態で、手首の左右往復運動（方向転換）が
min_duration_sec 秒間・min_swings 回以上続いたら発火する。
「右手を上げて静止」= FORWARD テレオペ姿勢とは、往復運動の有無で区別する。

テレオペとの共存:
- 振っている間（直近 max_gap_sec 内に方向転換あり）は is_active を立て、
  呼び出し側で速度指令を 0 に抑制する（振り始めの一瞬に FORWARD が
  混じるのを debounce と合わせて防ぐ）。

安全要件（docs 開発注記 §6）: 持続条件（2 秒 + 方向転換 4 回）・cooldown・NO BODY リセット。
"""

from collections import deque
from dataclasses import dataclass

L_SHOULDER, R_SHOULDER = 11, 12
R_WRIST = 16
NUM_LANDMARKS = 33


@dataclass(frozen=True)
class WaveParams:
    """しきい値（距離は肩幅 sw 正規化・時間は秒）."""

    min_amplitude: float = 0.25   # 方向転換とみなす片振れ幅 [sw]（ジッタ除去）
    min_swings: int = 4           # 発火に必要な方向転換回数
    min_duration_sec: float = 2.0  # 振りの継続時間
    max_gap_sec: float = 0.8      # 方向転換の間隔がこれを超えたら振り終了
    cooldown_sec: float = 10.0    # 発火後の不応期


@dataclass(frozen=True)
class WaveStatus:
    waving: bool      # 振り検出中（テレオペ抑制すべき）
    progress: float   # 発火までの進捗 0.0〜1.0（表示用）
    triggered: bool   # このフレームで発火（cooldown 済み）


class WaveDetector:
    """右手首の左右往復運動を検出する状態機械."""

    def __init__(self, params: WaveParams = WaveParams()):
        self._p = params
        self._anchor = None   # 現在の振り方向での極値 [sw]
        self._dir = None      # +1=画面右へ移動中 / -1=左 / None=未確定
        self._swings = deque()  # 方向転換の時刻列
        self._last_trigger = None

    def reset(self) -> None:
        self._anchor = None
        self._dir = None
        self._swings.clear()

    def update(self, lm_list, w, h, now: float) -> WaveStatus:
        if len(lm_list) < NUM_LANDMARKS:
            self.reset()
            return WaveStatus(False, 0.0, False)

        def norm(idx):
            return lm_list[idx][1] / float(w), lm_list[idx][2] / float(h)

        lsx, _ = norm(L_SHOULDER)
        rsx, rsy = norm(R_SHOULDER)
        rwx, rwy = norm(R_WRIST)
        sw = abs(lsx - rsx) or 1e-6

        # 「手を上げている」= 右手首が右肩ラインより上（FORWARD の raise より緩い。
        #  顔の横で振る高さを含める）
        if rwy >= rsy:
            self.reset()
            return WaveStatus(False, 0.0, False)

        # 振りの間隔が空いたら振り終了
        if self._swings and now - self._swings[-1] > self._p.max_gap_sec:
            self.reset()

        x = (rwx - rsx) / sw
        amp = self._p.min_amplitude
        if self._anchor is None:
            self._anchor = x
        if self._dir is None:
            if x - self._anchor >= amp:
                self._dir = 1
                self._swings.append(now)
                self._anchor = x
            elif self._anchor - x >= amp:
                self._dir = -1
                self._swings.append(now)
                self._anchor = x
        elif self._dir > 0:
            self._anchor = max(self._anchor, x)
            if self._anchor - x >= amp:   # 折り返し（右→左）
                self._dir = -1
                self._swings.append(now)
                self._anchor = x
        else:
            self._anchor = min(self._anchor, x)
            if x - self._anchor >= amp:   # 折り返し（左→右）
                self._dir = 1
                self._swings.append(now)
                self._anchor = x

        waving = bool(self._swings) and now - self._swings[-1] <= self._p.max_gap_sec
        duration = (now - self._swings[0]) if self._swings else 0.0
        progress = 0.0
        if waving:
            progress = min(1.0, min(duration / self._p.min_duration_sec,
                                    len(self._swings) / float(self._p.min_swings)))

        triggered = False
        if (waving and duration >= self._p.min_duration_sec and
                len(self._swings) >= self._p.min_swings):
            in_cooldown = (self._last_trigger is not None and
                           now - self._last_trigger < self._p.cooldown_sec)
            if not in_cooldown:
                triggered = True
                self._last_trigger = now
            self.reset()  # 発火可否によらず振りを畳む（cooldown 中の連続発火防止）
        return WaveStatus(waving, progress, triggered)
