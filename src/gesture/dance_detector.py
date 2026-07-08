"""ダンス検出（純ロジック・ROS 非依存）.

「右手と左手を交互に伸び縮みさせる動きが min_duration_sec 秒間続いた」ことを
検出する。gesture_mapper と同じく ROS に依存させず、単体テスト可能に保つ
（docs/architecture.md の設計方針 2）。

旧実装（両手首の運動エネルギー方式）は腕振り全般で発火し誤認識が多かったため、
特定の振り付け（交互の腕伸ばし）だけに反応するパターン方式へ変更（2026-07-06）。

判定の考え方（docs/gesture_teleop_development_notes.md §6.1）:
- 「伸ばしている」= 手首が自分の肩から水平方向に extend_ratio*肩幅 以上離れている
  （右手は画面左方向へ、左手は画面右方向へ。肩幅 sw 正規化で距離・体格に非依存）。
- 「縮めている」= retract_ratio*肩幅 未満（ヒステリシスで境界のばたつきを防ぐ）。
- 「右だけ伸びている」→「左だけ伸びている」→ … の切り替わり（スワップ）が
  max_interval_sec 以内の間隔で連続し、その連なりが min_duration_sec 以上続き、
  かつスワップ回数が min_swaps 以上になったら発火。
- 発火後は cooldown_sec 秒間、再発火しない（§6.2 debounce/cooldown）。
- NO BODY（ランドマーク不足）で状態をリセットする。

テレオペとの共存:
- 横に伸ばした腕は旋回ジェスチャーと同じ姿勢のため、交互パターンの進行中
  （スワップが 1 回以上起きて連なりが生きている間）は速度指令を 0 に抑制する
  is_active を返す（Move と Dance の同時発行を防ぐ。§6.3）。
"""

from dataclasses import dataclass

# gesture_mapper と同じランドマーク ID / 必要数
L_SHOULDER, R_SHOULDER = 11, 12
L_WRIST, R_WRIST = 15, 16
NUM_LANDMARKS = 33


@dataclass(frozen=True)
class DanceParams:
    """交互腕伸ばし検出のしきい値（距離は肩幅 sw 正規化・時間は秒）."""

    extend_ratio: float = 1.0     # 肩から sw*この値 以上 横に離れたら「伸ばした」
    retract_ratio: float = 0.6    # sw*この値 未満に戻ったら「縮めた」（ヒステリシス）
    min_duration_sec: float = 5.0  # 交互パターンがこの秒数続いたら発火
    min_swaps: int = 4            # 発火に必要な左右切り替わり回数（5 秒間の下限）
    max_interval_sec: float = 2.0  # 切り替わりの間隔がこれを超えたらパターン中断
    cooldown_sec: float = 15.0    # 発火後の不応期（Dance 動作の完了時間以上に）
    no_body_grace_sec: float = 0.7  # NO BODY がこの秒数以内ならチェーンを維持
                                    # （13fps 実機で 1 フレームの欠損が頻発するため。
                                    #   超えたら従来どおり全リセット）


@dataclass(frozen=True)
class DanceStatus:
    """update() の結果."""

    swaps: int           # 現在の連なりの左右切り替わり回数
    is_active: bool      # テレオペ出力を 0 に抑制すべきか（パターン進行中）
    triggered: bool      # このフレームでダンス発火（cooldown 済み）
    progress: float      # 発火までの進捗 0.0〜1.0（表示用）


class DanceDetector:
    """「右手と左手の交互伸び縮み」を検出する状態機械."""

    def __init__(self, params: DanceParams = DanceParams()):
        self._p = params
        self._r_extended = False   # 各腕の伸ばし状態（ヒステリシス付き）
        self._l_extended = False
        self._last_side = None     # 直近に「片side だけ伸びていた」のはどちらか
        self._chain_start = None   # 交互パターンの連なりの開始時刻
        self._last_swap = None     # 最後に切り替わった時刻
        self._swaps = 0
        self._last_trigger = None
        self._missing_since = None  # NO BODY が始まった時刻（grace 判定用）

    def reset(self) -> None:
        self._r_extended = False
        self._l_extended = False
        self._last_side = None
        self._chain_start = None
        self._last_swap = None
        self._swaps = 0
        self._missing_since = None

    def update(self, lm_list, w, h, now: float) -> DanceStatus:
        """1 フレーム分のランドマークを与えて状態を更新する.

        Args:
            lm_list: PoseTracker.find_position() が返す [id, x, y]（ピクセル）。
            w, h: フレーム幅・高さ。
            now: 現在時刻 [秒]（単調増加なら基準は問わない）。
        """
        if len(lm_list) < NUM_LANDMARKS:
            if self._chain_start is None:
                # チェーン未開始なら即リセット（従来どおり）
                self.reset()
                return DanceStatus(0, False, False, 0.0)
            # チェーン進行中の短時間欠損は許容する（grace 超過で全リセット）
            if self._missing_since is None:
                self._missing_since = now
            if now - self._missing_since > self._p.no_body_grace_sec:
                self.reset()
                return DanceStatus(0, False, False, 0.0)
            duration = now - self._chain_start
            progress = min(1.0, min(duration / self._p.min_duration_sec,
                                    self._swaps / float(self._p.min_swaps)))
            return DanceStatus(self._swaps, self._swaps >= 1, False, progress)
        self._missing_since = None

        def norm(idx):
            return lm_list[idx][1] / float(w), lm_list[idx][2] / float(h)

        lsx, _ = norm(L_SHOULDER)
        rsx, _ = norm(R_SHOULDER)
        lwx, _ = norm(L_WRIST)
        rwx, _ = norm(R_WRIST)
        sw = abs(lsx - rsx) or 1e-6

        # 伸ばし判定（ヒステリシス）。画面左側＝人物の右（gesture_mapper と同じ座標系）。
        # 右手: 右肩から画面左方向（x が小さい方）へ / 左手: 左肩から画面右方向へ。
        r_off = (rsx - rwx) / sw   # 正 = 右方向へ伸びている
        l_off = (lwx - lsx) / sw   # 正 = 左方向へ伸びている
        self._r_extended = r_off >= (self._p.retract_ratio if self._r_extended
                                     else self._p.extend_ratio)
        self._l_extended = l_off >= (self._p.retract_ratio if self._l_extended
                                     else self._p.extend_ratio)

        # 「片方だけ伸びている」ときだけ side を確定（両方/両方なしは中立）
        side = None
        if self._r_extended and not self._l_extended:
            side = "R"
        elif self._l_extended and not self._r_extended:
            side = "L"

        # 切り替わりの間隔が空きすぎたらパターン中断
        if self._last_swap is not None and now - self._last_swap > self._p.max_interval_sec:
            self._last_side = None
            self._chain_start = None
            self._last_swap = None
            self._swaps = 0

        if side is not None and side != self._last_side:
            if self._last_side is None:
                # 連なりの起点（最初の伸ばし）。スワップにはまだ数えない。
                if self._chain_start is None:
                    self._chain_start = now
            else:
                self._swaps += 1
            self._last_side = side
            self._last_swap = now

        active = self._chain_start is not None and self._swaps >= 1
        duration = (now - self._chain_start) if self._chain_start is not None else 0.0
        progress = 0.0
        if self._chain_start is not None:
            progress = min(1.0, min(duration / self._p.min_duration_sec,
                                    self._swaps / float(self._p.min_swaps)))

        triggered = False
        if (self._chain_start is not None and
                duration >= self._p.min_duration_sec and
                self._swaps >= self._p.min_swaps):
            in_cooldown = (self._last_trigger is not None and
                           now - self._last_trigger < self._p.cooldown_sec)
            if not in_cooldown:
                triggered = True
                self._last_trigger = now
            # 発火可否によらず連なりを畳む（踊り続けても cooldown 中は再発火しない）
            self._chain_start = None
            self._last_side = None
            self._last_swap = None
            self._swaps = 0

        return DanceStatus(self._swaps, active, triggered, progress)
