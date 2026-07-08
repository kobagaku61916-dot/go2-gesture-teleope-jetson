"""人追従コントローラ（純ロジック・ROS 非依存）.

MediaPipe Pose の両肩から「人の横位置 px」と「肩幅 sw」を取り、
- 旋回: 人を画面中央に保つ（omega = k_yaw × 中心からのずれ）
- 前後: 肩幅を目標値に保つ（sw は距離に反比例。遠い→前進 / 近すぎ→後退）
の P 制御で (vx, omega) を出す。距離センサ不要（深度への置き換えは将来課題）。

安全設計（gesture_teleop_development_notes §6 の精神を踏襲）:
- 人を見失ったら（NO BODY / 低信頼は呼び出し側で空リスト化）即 0 を返す。
  **探索行動はしない**（勝手に回って人を探さない）
- 目標より近い場合は後退（上限 max_back_vx）で距離を取り戻す
- デッドバンド（目標±sw_deadband・中心±center_deadband）で静止時のハンチングを防ぐ
- 出力は max_vx / max_omega で制限（さらに safety_gate のクランプ 0.6/0.8 が最終防壁）
- ローパスで急変を抑制。見失い→再検出時はローパス状態をリセット（古い指令を引きずらない）

キャリブレーション:
- target_sw は「追従したい距離に人が立ったときの正規化肩幅」。既定 0.19 は
  D435i HFOV≈69°・肩幅 0.4m・距離 1.5m からの幾何計算による推定値。
  実機では Phase A のログで実測して configs を上書きすること。
"""

from dataclasses import dataclass

L_SHOULDER, R_SHOULDER = 11, 12
NUM_LANDMARKS = 33


@dataclass(frozen=True)
class FollowParams:
    """追従制御パラメータ（距離はすべて正規化肩幅・速度は m/s, rad/s）."""

    target_sw: float = 0.19       # 目標肩幅（= 追従距離 1.5m 相当の推定値。要実測校正）
    sw_deadband: float = 0.10     # 目標 sw ±この比率以内は前後停止
    center_deadband: float = 0.05  # 画面中心 ±この範囲は旋回停止
    k_dist: float = 4.0           # 前後 P ゲイン [m/s / sw誤差]
    k_yaw: float = 2.0            # 旋回 P ゲイン [rad/s / 正規化xずれ]
    max_vx: float = 0.3           # 前進上限（safety_gate の 0.6 よりさらに保守的）
    max_back_vx: float = 0.2      # 後退上限
    max_omega: float = 0.6        # 旋回上限
    smooth_alpha: float = 0.4     # 出力ローパス（新値の重み。1.0 でフィルタなし）


@dataclass(frozen=True)
class FollowStatus:
    vx: float
    omega: float
    label: str      # 表示・ログ用: NO TARGET / FOLLOW / TOO CLOSE / HOLD
    sw: float       # 現在の正規化肩幅（キャリブレーション用）


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FollowController:
    """肩幅ベースの人追従 P 制御（ローパス付き状態機械）."""

    def __init__(self, params: FollowParams = FollowParams()):
        self._p = params
        self._vx = 0.0
        self._omega = 0.0

    def reset(self) -> None:
        self._vx = 0.0
        self._omega = 0.0

    def update(self, lm_list, w, h) -> FollowStatus:
        """1 フレーム分のランドマークから追従指令を返す.

        Args:
            lm_list: PoseTracker.find_position() の [id, x, y]（低信頼時は
                呼び出し側で空リストにしてから渡すこと）。
            w, h: フレーム幅・高さ。
        """
        p = self._p
        if len(lm_list) < NUM_LANDMARKS:
            # 見失い: 即停止（ローパスも通さない）・状態リセット
            self.reset()
            return FollowStatus(0.0, 0.0, "NO TARGET", 0.0)

        lsx = lm_list[L_SHOULDER][1] / float(w)
        rsx = lm_list[R_SHOULDER][1] / float(w)
        px = (lsx + rsx) / 2.0          # 人の横位置（0..1）
        sw = abs(lsx - rsx) or 1e-6     # 正規化肩幅（距離の代理）

        # --- 前後: 肩幅を目標に保つ ---
        err = p.target_sw - sw          # 正 = 遠い（前進）/ 負 = 近い（後退）
        if abs(err) <= p.target_sw * p.sw_deadband:
            vx_raw = 0.0
        else:
            vx_raw = _clamp(p.k_dist * err, -p.max_back_vx, p.max_vx)

        # --- 旋回: 人を画面中央に保つ（カメラ正面向き。ミラーは適用しない）---
        err_x = 0.5 - px                # 正 = 人が画面左 = ロボットの左 → 左旋回(+z)
        if abs(err_x) <= p.center_deadband:
            omega_raw = 0.0
        else:
            omega_raw = _clamp(p.k_yaw * err_x, -p.max_omega, p.max_omega)

        # --- ローパス（急変抑制）---
        a = p.smooth_alpha
        self._vx = a * vx_raw + (1 - a) * self._vx
        self._omega = a * omega_raw + (1 - a) * self._omega
        # 微小値は 0 に落とす（漸近で残る微速のカット）
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
        return FollowStatus(self._vx, self._omega, label, sw)
