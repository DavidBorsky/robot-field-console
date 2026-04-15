"""Camera abstraction layer.

The robot is expected to use a Logitech camera connected to the Raspberry Pi.
This file provides a simulated camera for development now and a hardware
camera wrapper that prefers OpenCV when available, then falls back to common
command-line capture tools on older Pi images.
"""

import shutil
import subprocess
import tempfile
import threading
import time
from pathlib import Path

try:
    from typing import Optional, Protocol, Sequence
except ImportError:  # pragma: no cover - Python < 3.8 fallback
    from typing import Optional, Sequence

    class Protocol(object):
        pass


class CameraFrame:
    def __init__(self, frame_id, width, height, source, frame_data=None, jpeg_bytes=None):
        self.frame_id = frame_id
        self.width = width
        self.height = height
        self.source = source
        self.frame_data = frame_data
        self.jpeg_bytes = jpeg_bytes


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


class CameraStatus:
    def __init__(self, connected, mode, detail, index=None):
        self.connected = connected
        self.mode = mode
        self.detail = detail
        self.index = index


class Camera(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def read_frame(self) -> CameraFrame: ...
    def get_target(self) -> VisionTarget: ...
    def status(self) -> CameraStatus: ...


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

    def status(self) -> CameraStatus:
        detail = "simulated Logitech-compatible camera feed"
        return CameraStatus(connected=self.running, mode="simulated", detail=detail, index=None)


class HardwareCamera:
    """Real Logitech camera implementation using OpenCV."""

    def __init__(
        self,
        index: Optional[int] = None,
        candidate_indices: Sequence[int] = (0, 1, 2, 3),
    ):
        self.index = index
        self.candidate_indices = tuple(candidate_indices)
        self.running = False
        self.cap = None
        self.cv2 = None
        self.frame_id = 0
        self.last_error = ""
        self.capture_backend = None
        self.backend_detail = ""
        self.command_width = 320
        self.command_height = 240
        self.command_device = "/dev/video0"
        self.command_interval_s = 0.2
        self._latest_command_jpeg_bytes = None
        self._latest_command_frame_id = 0
        self._command_lock = threading.Lock()
        self._command_capture_thread = None
        self._command_stop_event = threading.Event()

    def _indices_to_try(self):
        if self.index is not None and self.index >= 0:
            return (self.index,)
        return self.candidate_indices

    def start(self) -> None:
        try:
            import cv2
            self.cv2 = cv2
        except ImportError:
            self.cv2 = None

        if self.cv2 is not None:
            self.last_error = ""
            attempted = []
            for camera_index in self._indices_to_try():
                attempted.append(camera_index)
                cap = self.cv2.VideoCapture(camera_index)
                if cap is None or not cap.isOpened():
                    if cap is not None:
                        cap.release()
                    self.last_error = "camera {} did not open".format(camera_index)
                    continue

                ret, _frame = cap.read()
                if not ret:
                    cap.release()
                    self.last_error = "camera {} opened but did not return frames".format(camera_index)
                    continue

                self.cap = cap
                self.index = camera_index
                self.running = True
                self.frame_id = 0
                self.last_error = ""
                self.capture_backend = "opencv"
                self.backend_detail = "OpenCV VideoCapture"
                return

            attempted_display = ", ".join(str(value) for value in attempted)
            self.last_error = (
                "Could not open any camera device (tried: {}). {}".format(
                    attempted_display,
                    self.last_error or "no camera indices were attempted",
                )
            )

        if self._command_backend_available():
            self.running = True
            self.frame_id = 0
            self.capture_backend = "command"
            self._command_stop_event.clear()
            self._command_capture_thread = threading.Thread(
                target=self._command_capture_loop,
                name="camera-command-capture",
                daemon=True,
            )
            self._command_capture_thread.start()
            if not self.last_error:
                self.last_error = ""
            return

        self.running = False
        if not self.last_error:
            self.last_error = "OpenCV not available and no command-line camera backend was found"
        raise RuntimeError(self.last_error)

    def _command_backend_available(self) -> bool:
        fswebcam = self._find_command("fswebcam")
        if fswebcam:
            self.backend_detail = fswebcam
            return True
        self.last_error = "OpenCV not available and fswebcam was not found on PATH"
        return False

    def _find_command(self, name):
        return shutil.which(name)

    def stop(self) -> None:
        self.running = False
        self._command_stop_event.set()
        if self._command_capture_thread is not None:
            self._command_capture_thread.join(timeout=1.0)
            self._command_capture_thread = None
        if self.cap:
            self.cap.release()
            self.cap = None
        self.cv2 = None
        self.capture_backend = None

    def read_frame(self) -> CameraFrame:
        if not self.running or not self.cap or self.cv2 is None:
            if self.capture_backend == "command":
                with self._command_lock:
                    if self._latest_command_jpeg_bytes:
                        return CameraFrame(
                            frame_id=self._latest_command_frame_id,
                            width=self.command_width,
                            height=self.command_height,
                            source="camera-command",
                            frame_data=None,
                            jpeg_bytes=self._latest_command_jpeg_bytes,
                        )
                return self._read_frame_from_command()
            raise RuntimeError("HardwareCamera is not running")

        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from camera")

        height, width = frame.shape[:2]
        self.frame_id += 1
        return CameraFrame(
            frame_id=self.frame_id,
            width=width,
            height=height,
            source="camera-{}".format(self.index),
            frame_data=frame
        )

    def _read_frame_from_command(self) -> CameraFrame:
        frame_id, jpeg_bytes = self._capture_frame_from_command()
        return CameraFrame(
            frame_id=frame_id,
            width=self.command_width,
            height=self.command_height,
            source="camera-command",
            frame_data=None,
            jpeg_bytes=jpeg_bytes,
        )

    def _command_capture_loop(self) -> None:
        while not self._command_stop_event.is_set():
            try:
                frame_id, jpeg_bytes = self._capture_frame_from_command()
                with self._command_lock:
                    self._latest_command_frame_id = frame_id
                    self._latest_command_jpeg_bytes = jpeg_bytes
            except Exception as exc:
                self.last_error = str(exc)
            self._command_stop_event.wait(self.command_interval_s)

    def _capture_frame_from_command(self):
        fswebcam = self._find_command("fswebcam")
        if not fswebcam:
            raise RuntimeError("fswebcam is not installed")

        palette_attempts = [None, "MJPEG", "YUYV", "UYVY", "JPEG"]
        last_error = "unknown fswebcam error"

        for palette in palette_attempts:
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as handle:
                temp_path = Path(handle.name)

            try:
                command = [
                    fswebcam,
                    "-q",
                    "--no-banner",
                    "-d",
                    self.command_device,
                    "--jpeg",
                    "50",
                    "-r",
                    "{}x{}".format(self.command_width, self.command_height),
                ]
                if palette is not None:
                    command.extend(["--palette", palette])
                command.append(str(temp_path))
                result = subprocess.run(
                    command,
                    check=False,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    universal_newlines=True,
                )
                if result.returncode != 0:
                    detail = result.stderr.strip() or result.stdout.strip() or "unknown fswebcam error"
                    last_error = "{}{}".format(
                        detail,
                        "" if palette is None else " (palette {})".format(palette),
                    )
                    continue

                jpeg_bytes = temp_path.read_bytes()
                if not jpeg_bytes:
                    last_error = "fswebcam produced an empty image{}".format(
                        "" if palette is None else " (palette {})".format(palette),
                    )
                    continue

                self.frame_id += 1
                self.backend_detail = "{} via {}".format(
                    fswebcam,
                    "default palette" if palette is None else "{} palette".format(palette),
                )
                self.last_error = ""
                return self.frame_id, jpeg_bytes
            finally:
                try:
                    temp_path.unlink()
                except Exception:
                    pass

        raise RuntimeError("fswebcam capture failed: {}".format(last_error))

    def get_target(self) -> VisionTarget:
        # TODO: Implement vision processing to detect targets
        return VisionTarget(visible=False)

    def status(self) -> CameraStatus:
        if self.running:
            if self.capture_backend == "opencv":
                detail = "connected to camera {} via {}".format(self.index, self.backend_detail)
            else:
                with self._command_lock:
                    has_frame = self._latest_command_jpeg_bytes is not None
                detail = "connected via {}".format(self.backend_detail or "command backend")
                if not has_frame and self.last_error:
                    detail = self.last_error
            return CameraStatus(connected=True, mode="hardware", detail=detail, index=self.index)

        detail = self.last_error or "camera not started"
        return CameraStatus(connected=False, mode="hardware", detail=detail, index=self.index)


class UnavailableCamera:
    def __init__(self, detail: str):
        self.detail = detail

    def start(self) -> None:
        return

    def stop(self) -> None:
        return

    def read_frame(self) -> CameraFrame:
        raise RuntimeError(self.detail)

    def get_target(self) -> VisionTarget:
        return VisionTarget(visible=False)

    def status(self) -> CameraStatus:
        return CameraStatus(connected=False, mode="unavailable", detail=self.detail, index=None)


def create_camera(simulate: bool = True, index: Optional[int] = None) -> Camera:
    if simulate:
        return SimulatedCamera()
    hardware_index = None if index is None or index < 0 else index
    return HardwareCamera(index=hardware_index)
