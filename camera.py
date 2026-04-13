"""Camera abstraction layer.

The robot is expected to use a Logitech camera connected to the Raspberry Pi.
This file provides a simulated camera for development now and a placeholder
hardware camera wrapper for later.
"""

try:
    from typing import Protocol
except ImportError:  # pragma: no cover - Python < 3.8 fallback
    class Protocol(object):
        pass


class CameraFrame:
    def __init__(self, frame_id, width, height, source, frame_data=None):
        self.frame_id = frame_id
        self.width = width
        self.height = height
        self.source = source
        self.frame_data = frame_data


class VisionTarget:
    def __init__(
        self,
        visible,
        x_offset_norm=0.0,
        y_offset_norm=0.0,
        confidence=0.0,
        label="",
    ):
        self.visible = visible
        self.x_offset_norm = x_offset_norm
        self.y_offset_norm = y_offset_norm
        self.confidence = confidence
        self.label = label


class Camera(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def read_frame(self) -> CameraFrame: ...
    def get_target(self) -> VisionTarget: ...


class SimulatedCamera:
    """Simple development camera that can pretend a target is visible."""

    def __init__(self):
        self.running = False
        self.frame_id = 0
        self.target = VisionTarget(visible=False)

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def set_target(self, target: VisionTarget) -> None:
        self.target = target

    def read_frame(self) -> CameraFrame:
        if not self.running:
            raise RuntimeError("SimulatedCamera is not running")
        self.frame_id += 1
        return CameraFrame(
            frame_id=self.frame_id,
            width=640,
            height=480,
            source="simulated-logitech",
            frame_data=None  # No actual frame data for simulation
        )

    def get_target(self) -> VisionTarget:
        return self.target


class HardwareCamera:
    """Real Logitech camera implementation using OpenCV."""

    def __init__(self, index: int = 0):
        self.index = index
        self.running = False
        self.cap = None
        self.cv2 = None

    def start(self) -> None:
        try:
            import cv2
            self.cv2 = cv2
            self.cap = cv2.VideoCapture(self.index)
            if not self.cap.isOpened():
                raise RuntimeError("Could not open camera {}".format(self.index))
            self.running = True
        except ImportError:
            raise RuntimeError("OpenCV not available. Install with: pip install opencv-python")

    def stop(self) -> None:
        self.running = False
        if self.cap:
            self.cap.release()
            self.cap = None
        self.cv2 = None

    def read_frame(self) -> CameraFrame:
        if not self.running or not self.cap or self.cv2 is None:
            raise RuntimeError("HardwareCamera is not running")

        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from camera")

        height, width = frame.shape[:2]
        return CameraFrame(
            frame_id=int(self.cap.get(self.cv2.CAP_PROP_POS_FRAMES)),
            width=width,
            height=height,
            source="camera-{}".format(self.index),
            frame_data=frame
        )

    def get_target(self) -> VisionTarget:
        # TODO: Implement vision processing to detect targets
        return VisionTarget(visible=False)


def create_camera(simulate: bool = True) -> Camera:
    if simulate:
        return SimulatedCamera()
    return HardwareCamera()
