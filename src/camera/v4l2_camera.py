"""V4L2 (USB UVC) カメラ実装.

Linux ネイティブで /dev/videoN を OpenCV (CAP_V4L2) で直接開く。
旧リポの WSL2 対策（MJPG 強制・フレーム読み捨て）は不要のため持ち込まない
（2026-07-02 実測: RealSense D435i の RGB=/dev/video4 は素の YUYV で正常取得）。

device 番号はハードコードせず、必ず設定（params.yaml / -p camera.device:=N）から
与えること。RealSense D435i では RGB が /dev/video4（video0 は深度で開けない）。
"""

import cv2

from .base import CameraSource


class V4L2Camera(CameraSource):
    """OpenCV VideoCapture (V4L2) による USB カメラ入力."""

    def __init__(self, device: int, width: int = 640, height: int = 480,
                 fourcc: str = ""):
        """
        Args:
            device: /dev/videoN の N。
            width, height: 要求解像度（デバイスが拒否したら近い値になる）。
            fourcc: 4 文字のピクセルフォーマット指定（例 "MJPG"）。
                空文字ならデバイス既定（通常 YUYV）を使う。
        """
        self._device = int(device)
        self._width = int(width)
        self._height = int(height)
        self._fourcc = fourcc
        self._cap = None

    def open(self) -> bool:
        cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False
        if self._fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        ok, _ = cap.read()
        if not ok:
            cap.release()
            return False
        self._cap = cap
        return True

    def read(self):
        if self._cap is None:
            return False, None
        return self._cap.read()

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    @property
    def actual_size(self):
        """実際に設定された (width, height)。open 後に有効."""
        if self._cap is None:
            return None
        return (int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def __repr__(self):
        return (f"V4L2Camera(device={self._device}, "
                f"{self._width}x{self._height}, fourcc={self._fourcc or 'default'})")
