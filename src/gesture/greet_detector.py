"""Greet（あいさつ）検出 — 「両手をカメラに向かってかざす」ポーズ（純ロジック・ROS 非依存）.

両手首が両肩より上（gesture_mapper の raise と同じ正規化条件を両腕同時に満たす）を
hold_sec 秒間保持したら発火する。片手だけなら FORWARD/BACKWARD のテレオペ姿勢なので
発火しない。gesture_mapper 側では「両手上げ」は速度 0（STOP）になるため、
テレオペ指令との競合は構造的に発生しない。

安全要件（docs 開発注記 §6）:
- 静的ポーズの一定時間保持（hold_sec）ではじめて発火（§6.1）
- 発火後は cooldown_sec の不応期（§6.2）
- NO BODY でリセット
"""

from dataclasses import dataclass

L_SHOULDER, R_SHOULDER = 11, 12
L_WRIST, R_WRIST = 15, 16
NUM_LANDMARKS = 33


@dataclass(frozen=True)
class GreetParams:
    """しきい値（gesture_mapper の raise 判定と同じ正規化・既定値）."""

    raise_margin: float = 0.5   # 手首が肩より sw*この値 以上 上
    raise_near: float = 1.2    # 肩の真上付近（横ズレが sw*この値 以内）
    hold_sec: float = 1.5      # この秒数保持で発火
    cooldown_sec: float = 10.0  # 発火後の不応期


@dataclass(frozen=True)
class GreetStatus:
    holding: bool     # 両手かざしポーズを保持中か
    progress: float   # 発火までの進捗 0.0〜1.0（表示用）
    triggered: bool   # このフレームで発火（cooldown 済み）


class GreetDetector:
    """「両手を上げてかざす」ポーズの保持を検出する."""

    def __init__(self, params: GreetParams = GreetParams()):
        self._p = params
        self._since = None
        self._last_trigger = None

    def reset(self) -> None:
        self._since = None

    def update(self, lm_list, w, h, now: float) -> GreetStatus:
        if len(lm_list) < NUM_LANDMARKS:
            self.reset()
            return GreetStatus(False, 0.0, False)

        def norm(idx):
            return lm_list[idx][1] / float(w), lm_list[idx][2] / float(h)

        lsx, lsy = norm(L_SHOULDER)
        rsx, rsy = norm(R_SHOULDER)
        lwx, lwy = norm(L_WRIST)
        rwx, rwy = norm(R_WRIST)
        sw = abs(lsx - rsx) or 1e-6

        right_raised = (rwy < rsy - self._p.raise_margin * sw) and \
                       (abs(rwx - rsx) < self._p.raise_near * sw)
        left_raised = (lwy < lsy - self._p.raise_margin * sw) and \
                      (abs(lwx - lsx) < self._p.raise_near * sw)
        holding = right_raised and left_raised

        if not holding:
            self._since = None
            return GreetStatus(False, 0.0, False)

        if self._since is None:
            self._since = now
        held = now - self._since
        progress = min(1.0, held / self._p.hold_sec)

        triggered = False
        if held >= self._p.hold_sec:
            in_cooldown = (self._last_trigger is not None and
                           now - self._last_trigger < self._p.cooldown_sec)
            if not in_cooldown:
                triggered = True
                self._last_trigger = now
            self._since = None  # 発火後は保持し直し
        return GreetStatus(True, progress, triggered)
