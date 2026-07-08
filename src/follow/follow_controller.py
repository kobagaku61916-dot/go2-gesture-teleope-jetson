"""人追従コントローラ v2（純ロジック・ROS 非依存）.

MediaPipe Pose の両肩から「人の横位置 px」と「肩幅 sw」を取り、
- 距離推定: dist ≈ target_distance_m × sw_at_target / sw（sw は距離に反比例）
- 前後: 距離誤差[m]の P 制御（遠いほど速く。上限 max_vx / 近すぎは後退）
- 旋回: 人を画面中央に保つ P 制御
で (vx, omega) を出す。

v1（sw 誤差の線形 P）からの改良（2026-07-09 Phase B 実機フィードバック）:
- sw は距離に反比例するため、sw 誤差ベースだと遠方で指令が伸びず「遅い」。
  距離[m]に変換してから P を掛けることで、離れるほど素直に加速する
- 見失い猶予 lost_grace_sec（既定 0.25s）を追加。1 フレームの検出落ちで
  指令がゼロに落ちてローパスが再立ち上がる「ストップ&ゴー」を解消する。
  猶予を超えたら完全停止（探索はしない）。視野外デッドマンは最大 0.25s 遅れるが
  0.6m/s でも +15cm であり、safety_gate の watchdog(0.5s) の内側に収まる

安全設計:
- 猶予超過の見失いで即 0・状態リセット（古い指令を引きずらない）
- 後退上限 max_back_vx は控えめ（ロボットは後方が見えない）
- 出力は max_vx / max_omega 制限 + safety_gate クランプ(0.6/0.8) の二重防壁
"""

from dataclasses import dataclass

L_SHOULDER, R_SHOULDER = 11, 12
NUM_LANDMARKS = 33


@dataclass(frozen=True)
class FollowParams:
    """追従制御パラメータ."""

    target_distance_m: float = 1.5  # 追従目標距離 [m]
    sw_at_target: float = 0.105     # 目標距離に立った人の正規化肩幅（実測校正値）
    deadband_m: float = 0.15        # 距離誤差 ±この範囲[m]は前後停止
    center_deadband: float = 0.05   # 画面中心 ±この範囲は旋回停止
    k_dist: float = 0.8             # 前後 P ゲイン [m/s per m]
    k_yaw: float = 2.0              # 旋回 P ゲイン [rad/s per 正規化xずれ]
    max_vx: float = 0.6             # 前進上限（= safety_gate クランプ上限）
    max_back_vx: float = 0.2        # 後退上限
    max_omega: float = 0.8          # 旋回上限
    smooth_alpha: float = 0.6       # 出力ローパス（新値の重み）
    lost_grace_sec: float = 0.25    # 見失い猶予（直前指令を保持する時間）
    max_est_distance_m: float = 6.0  # 距離推定の上限（異常な sw 値の暴走防止）


@dataclass(frozen=True)
class FollowStatus:
    vx: float
    omega: float
    label: str        # NO TARGET / FOLLOW / TOO CLOSE / HOLD / COAST(猶予中)
    distance_m: float  # 推定距離 [m]（見失い中は 0.0）


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FollowController:
    """距離推定ベースの人追従 P 制御（見失い猶予・ローパス付き）."""

    def __init__(self, params: FollowParams = FollowParams()):
        self._p = params
        self._vx = 0.0
        self._omega = 0.0
        self._last_seen = None   # 最後に人を検出した時刻

    def reset(self) -> None:
        self._vx = 0.0
        self._omega = 0.0
        self._last_seen = None

    def update(self, lm_list, w, h, now: float) -> FollowStatus:
        """1 フレーム分のランドマークから追従指令を返す.

        Args:
            lm_list: PoseTracker.find_position() の [id, x, y]（低信頼時は
                呼び出し側で空リストにしてから渡すこと）。
            w, h: フレーム幅・高さ。
            now: 現在時刻 [秒]（単調増加）。
        """
        p = self._p
        if len(lm_list) < NUM_LANDMARKS:
            if (self._last_seen is not None and
                    now - self._last_seen <= p.lost_grace_sec):
                # 猶予内: 直前指令を保持（1 フレーム落ちのストップ&ゴー防止）
                return FollowStatus(self._vx, self._omega, "COAST", 0.0)
            self.reset()
            return FollowStatus(0.0, 0.0, "NO TARGET", 0.0)
        self._last_seen = now

        lsx = lm_list[L_SHOULDER][1] / float(w)
        rsx = lm_list[R_SHOULDER][1] / float(w)
        px = (lsx + rsx) / 2.0
        sw = abs(lsx - rsx) or 1e-6

        # --- 距離推定と前後 P（メートル空間）---
        dist = _clamp(p.target_distance_m * p.sw_at_target / sw,
                      0.0, p.max_est_distance_m)
        err_m = dist - p.target_distance_m   # 正 = 遠い（前進）
        if abs(err_m) <= p.deadband_m:
            vx_raw = 0.0
        else:
            vx_raw = _clamp(p.k_dist * err_m, -p.max_back_vx, p.max_vx)

        # --- 旋回: 人を画面中央に保つ ---
        err_x = 0.5 - px
        if abs(err_x) <= p.center_deadband:
            omega_raw = 0.0
        else:
            omega_raw = _clamp(p.k_yaw * err_x, -p.max_omega, p.max_omega)

        # --- ローパス ---
        a = p.smooth_alpha
        self._vx = a * vx_raw + (1 - a) * self._vx
        self._omega = a * omega_raw + (1 - a) * self._omega
        if abs(self._vx) < 0.02:
            self._vx = 0.0
        if abs(self._omega) < 0.02:
            self._omega = 0.0

        if vx_raw < 0:
            label = "TOO CLOSE"
        elif vx_raw == 0.0 and omega_raw == 0.0:
            label = "HOLD"
        else:
            label = "FOLLOW"
        return FollowStatus(self._vx, self._omega, label, dist)
