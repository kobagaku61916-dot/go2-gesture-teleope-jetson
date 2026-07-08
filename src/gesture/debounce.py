"""速度指令の安定化 debounce（純ロジック・ROS 非依存）.

MediaPipe の 1 フレーム誤検出がそのまま Go2 の動きになるのを防ぐ
（architecture.md §4。旧リポで 1 フレームの誤指令が実機に届いた実測あり）。

規則:
- 非ゼロ指令は「同一の指令が debounce_frames フレーム連続」してはじめて通す。
  それまでは 0 を返す。
- ゼロ指令（STOP）は**即時**通す（停止を遅らせるのは安全上逆効果）。
"""

ZERO = (0.0, 0.0, 0.0)


class CommandDebouncer:
    """(vx, vy, omega) 指令の N フレーム連続一致フィルタ."""

    def __init__(self, debounce_frames: int = 3):
        if debounce_frames < 1:
            raise ValueError("debounce_frames は 1 以上")
        self._n = int(debounce_frames)
        self._last = ZERO
        self._count = 0

    def reset(self) -> None:
        self._last = ZERO
        self._count = 0

    def update(self, cmd) -> tuple:
        """1 フレーム分の指令を与え、通してよい指令を返す.

        Args:
            cmd: (vx, vy, omega) のタプル。

        Returns:
            安定化後の (vx, vy, omega)。
        """
        cmd = (float(cmd[0]), float(cmd[1]), float(cmd[2]))
        if cmd == ZERO:
            # STOP は即時。連続カウントもリセット
            self.reset()
            return ZERO
        if cmd == self._last:
            self._count += 1
        else:
            self._last = cmd
            self._count = 1
        return cmd if self._count >= self._n else ZERO
