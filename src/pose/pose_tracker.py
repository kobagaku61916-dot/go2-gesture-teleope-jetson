"""MediaPipe Pose ラッパ.

旧リポ go2_gesture_teleop の pose_tracker.py の移植（ロジック等価）。
mediapipe==**0.10.18** の solutions API を前提とする（aarch64 wheel の最新。
バージョン方針は requirements-jetson.md §3）。
本リポでの追加: find_visibilities()（信頼度チェック用。confidence.py 参照）。
"""

import cv2
import mediapipe as mp


class PoseTracker:
    """MediaPipe Pose をラップした全身姿勢検出クラス（33 ランドマーク）."""

    def __init__(self, static_mode: bool = False, model_complexity: int = 1,
                 smooth: bool = True, detection_confidence: float = 0.5,
                 tracking_confidence: float = 0.5):
        """
        Args:
            static_mode: 静止画モード。False なら動画用にトラッキング併用。
            model_complexity: 0=軽量 / 1=標準 / 2=高精度。
            smooth: ランドマークのフレーム間平滑化。
            detection_confidence: 検出と判断する信頼度しきい値。
            tracking_confidence: トラッキング継続の信頼度しきい値。
        """
        self._mp_pose = mp.solutions.pose
        self._pose = self._mp_pose.Pose(
            static_image_mode=static_mode,
            model_complexity=model_complexity,
            smooth_landmarks=smooth,
            min_detection_confidence=detection_confidence,
            min_tracking_confidence=tracking_confidence,
        )
        self._mp_draw = mp.solutions.drawing_utils
        self._results = None

    def find_pose(self, img, draw: bool = True):
        """画像から全身の姿勢を検出し、必要なら骨格を描画して返す.

        Args:
            img: BGR 形式の入力画像（OpenCV フレーム）。
            draw: True なら検出結果の骨格を img に描画する。

        Returns:
            （描画後の）画像。
        """
        # MediaPipe は RGB 入力前提のため BGR から変換する
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self._results = self._pose.process(img_rgb)

        if self._results.pose_landmarks and draw:
            # 関節は緑・骨格は黄で太めに描いて視認性を上げる（旧リポ踏襲）
            self._mp_draw.draw_landmarks(
                img,
                self._results.pose_landmarks,
                self._mp_pose.POSE_CONNECTIONS,
                self._mp_draw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=4),
                self._mp_draw.DrawingSpec(color=(0, 220, 255), thickness=3),
            )
        return img

    def find_position(self, img, draw: bool = False):
        """検出した全身ランドマークのピクセル座標リストを返す.

        Args:
            img: 座標計算の基準となる画像（サイズ取得に使用）。
            draw: True なら各ランドマーク位置に円を描画する。

        Returns:
            [ランドマークID, x, y] のリスト（最大 33 点）。
            未検出（find_pose 未実行含む）なら空リスト。
        """
        lm_list = []
        if self._results is not None and self._results.pose_landmarks:
            h, w = img.shape[:2]
            # 各ランドマークは 0.0〜1.0 の正規化座標 → ピクセル座標へ変換
            for lm_id, lm in enumerate(self._results.pose_landmarks.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                lm_list.append([lm_id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (0, 0, 255), cv2.FILLED)
        return lm_list

    def find_visibilities(self):
        """検出した全ランドマークの visibility（0.0〜1.0）リストを返す.

        Returns:
            ランドマーク ID を添字とする visibility のリスト。未検出なら空リスト。
            confidence.key_landmarks_visible() と組み合わせて使う。
        """
        if self._results is None or not self._results.pose_landmarks:
            return []
        return [lm.visibility for lm in self._results.pose_landmarks.landmark]

    def close(self) -> None:
        """MediaPipe のリソースを解放する."""
        self._pose.close()
