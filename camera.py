"""Camera abstraction layer.

The robot is expected to use a Logitech camera connected to the Raspberry Pi.
This file provides a simulated camera for development now and a placeholder
hardware camera wrapper for later.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class CameraFrame:
    frame_id: int
    width: int
    height: int
    source: str


@dataclass(frozen=True)
class VisionTarget:
    visible: bool
    x_offset_norm: float = 0.0
    y_offset_norm: float = 0.0
    confidence: float = 0.0
    label: str = ""


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
        )

    def get_target(self) -> VisionTarget:
        return self.target


class HardwareCamera:
    """Placeholder for the real Logitech camera implementation."""

    def __init__(self, index: int = 0):
        self.index = index
        self.running = False

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def read_frame(self) -> CameraFrame:
        if not self.running:
            raise RuntimeError("HardwareCamera is not running")
        return CameraFrame(frame_id=0, width=640, height=480, source=f"camera-{self.index}")

    def get_target(self) -> VisionTarget:
        return VisionTarget(visible=False)


def create_camera(simulate: bool = True) -> Camera:
    if simulate:
        return SimulatedCamera()
    return HardwareCamera()
