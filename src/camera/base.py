"""カメラ抽象化層のインターフェース定義.

ノード本体はこの CameraSource にのみ依存し、実体（USB/V4L2, RealSense, …）は
設定 camera.type で差し替える（docs/architecture.md「カメラ入力は差し替え可能に」）。
"""

from abc import ABC, abstractmethod


class CameraSource(ABC):
    """カメラ入力の共通インターフェース.

    使い方:
        cam = create_camera(...)   # camera/__init__.py のファクトリ
        if not cam.open():
            ...エラー処理...
        ok, frame = cam.read()     # frame は BGR (OpenCV 互換) の ndarray
        cam.close()
    """

    @abstractmethod
    def open(self) -> bool:
        """デバイスを開く。成功なら True."""

    @abstractmethod
    def read(self):
        """1 フレーム取得する。(ok: bool, frame: ndarray | None) を返す."""

    @abstractmethod
    def close(self) -> None:
        """デバイスを解放する（多重呼び出し可）."""

    # with 構文でも使えるように
    def __enter__(self):
        if not self.open():
            raise RuntimeError(f"{type(self).__name__}: カメラを開けませんでした")
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
