"""ジェスチャー判定（純関数）.

全身ランドマークから速度指令 (vx, vy, omega) とラベルを求める。
旧リポ gesture_teleope の gesture_ope.computeCommand() の移植。
ロジックは等価（test/test_gesture_mapper.py で保証）。

設計方針（docs/architecture.md）:
- ROS に依存しない純関数のまま維持する（単体テスト可能）。
- しきい値・速度は GestureParams で外から与える（ハードコードしない）。
- しきい値は肩幅 sw で正規化し、人物の大小・カメラ距離に依存しない。

座標系:
- 画面左上を原点 (0,0) とする正規化座標で判定。
- 映像は左右反転しない（仕様どおり「画面左側＝人物の右」になる）。

ジェスチャー仕様:
- 前進 (vx>0)    : 右手が右肩より高く垂直に上がっている
- 後進 (vx<0)    : 左手が左肩より高く垂直に上がっている
- 右旋回 (omega<0): 右手が右肩とほぼ同じ高さで水平に右方向（画面左側）へ伸びている
- 左旋回 (omega>0): 左手が左肩とほぼ同じ高さで水平に左方向（画面右側）へ伸びている
- 停止           : 両手が肩よりも下 / 全身が映っていない（NO BODY）
"""

from dataclasses import dataclass

# Pose のランドマーク ID（左右は人物基準＝解剖学的な左右）。
L_SHOULDER, R_SHOULDER = 11, 12
L_WRIST, R_WRIST = 15, 16

# MediaPipe Pose の全ランドマーク数。これに満たなければ NO BODY。
NUM_LANDMARKS = 33


@dataclass(frozen=True)
class GestureParams:
    """判定しきい値と速度の大きさ（既定値は旧リポの実績値）."""

    linear_speed: float = 0.5    # 前進・後進の速さ [m/s]
    angular_speed: float = 0.5   # 旋回の速さ [rad/s]
    # --- しきい値（肩幅 sw で正規化）---
    raise_margin: float = 0.5    # 手首が肩より「sw*この値」以上 上にあれば「上げている」
    raise_near: float = 1.2      # 上げた手が肩の真上付近（横ズレが sw*この値 以内）
    level_margin: float = 0.6    # 手首と肩の高さの差が sw*この値 以内なら「同じ高さ」
    extend_margin: float = 1.0   # 手首が肩から sw*この値 以上 横へ離れていれば「伸ばしている」


def compute_command(lm_list, w, h, params: GestureParams = GestureParams()):
    """全身ランドマークから速度指令 (vx, vy, omega) とラベルを求める.

    Args:
        lm_list: PoseTracker.find_position() が返す [id, x, y]（ピクセル）。
        w, h: フレーム幅・高さ（正規化に使う）。
        params: しきい値・速度設定。

    Returns:
        (vx, vy, omega, label)。ランドマーク不足時は (0,0,0,"NO BODY")。
    """
    if len(lm_list) < NUM_LANDMARKS:
        return 0.0, 0.0, 0.0, "NO BODY"

    # 画面左上を原点とする正規化座標 (0〜1) に変換する。
    def norm(idx):
        return lm_list[idx][1] / float(w), lm_list[idx][2] / float(h)

    lsx, lsy = norm(L_SHOULDER)   # 左肩
    rsx, rsy = norm(R_SHOULDER)   # 右肩
    lwx, lwy = norm(L_WRIST)      # 左手首
    rwx, rwy = norm(R_WRIST)      # 右手首

    # 肩幅（正規化）。スケール基準。0除算回避。
    sw = abs(lsx - rsx) or 1e-6

    # y は下ほど大きいので「上げている」= 手首の y が肩の y より小さい。
    # 右手を垂直に上げている（前進）: 肩より十分上、かつ肩の真上付近（横ズレ小）。
    right_raised = (rwy < rsy - params.raise_margin * sw) and \
                   (abs(rwx - rsx) < params.raise_near * sw)
    # 左手を垂直に上げている（後進）。
    left_raised = (lwy < lsy - params.raise_margin * sw) and \
                  (abs(lwx - lsx) < params.raise_near * sw)

    # 右手を肩の高さで右方向（画面左側＝x が小さい方）へ伸ばしている（右旋回）。
    right_extended = (abs(rwy - rsy) < params.level_margin * sw) and \
                     (rwx < rsx - params.extend_margin * sw)
    # 左手を肩の高さで左方向（画面右側＝x が大きい方）へ伸ばしている（左旋回）。
    left_extended = (abs(lwy - lsy) < params.level_margin * sw) and \
                    (lwx > lsx + params.extend_margin * sw)

    # 速度の決定。前進/後進（垂直）と旋回（水平）は独立に決め、組み合わせ可能。
    vx, vy, omega = 0.0, 0.0, 0.0
    if right_raised and not left_raised:
        vx = params.linear_speed          # 前進
    elif left_raised and not right_raised:
        vx = -params.linear_speed         # 後進

    if right_extended and not left_extended:
        omega = -params.angular_speed     # 右旋回
    elif left_extended and not right_extended:
        omega = params.angular_speed      # 左旋回

    # ラベル（表示用）。
    parts = []
    if vx > 0:
        parts.append("FORWARD")
    elif vx < 0:
        parts.append("BACKWARD")
    if omega < 0:
        parts.append("TURN-RIGHT")
    elif omega > 0:
        parts.append("TURN-LEFT")
    label = " + ".join(parts) if parts else "STOP"
    return vx, vy, omega, label
