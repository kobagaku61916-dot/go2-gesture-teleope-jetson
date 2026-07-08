"""カメラ抽象化層: 設定値からカメラ実装を生成するファクトリ."""

from .base import CameraSource
from .v4l2_camera import V4L2Camera

__all__ = ["CameraSource", "V4L2Camera", "create_camera"]


def create_camera(camera_type: str, device: int, width: int, height: int,
                  fourcc: str = "") -> CameraSource:
    """camera.type 設定値に応じた CameraSource を生成する.

    Args:
        camera_type: "v4l2"（USB/UVC。RealSense D435i の RGB もこれで可）。
            "realsense" は深度を使う段で pyrealsense2 実装を追加予定。
        device: /dev/videoN の N（v4l2 のとき）。
        width, height, fourcc: V4L2Camera 参照。
    """
    t = camera_type.strip().lower()
    if t == "v4l2":
        return V4L2Camera(device=device, width=width, height=height, fourcc=fourcc)
    if t == "realsense":
        raise NotImplementedError(
            "camera.type=realsense は未実装（pyrealsense2 で深度を使う段で追加予定）。"
            "D435i の RGB だけなら camera.type=v4l2 + camera.device=4 を使ってください。")
    raise ValueError(f"未知の camera.type: {camera_type!r}（対応: v4l2）")
