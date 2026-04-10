"""Gyro abstraction layer.

This starts with a simulated gyro for development and a placeholder hardware
implementation for when the real sensor is wired in.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class GyroReading:
    heading_deg: float
    angular_rate_dps: float = 0.0
    calibrated: bool = True


class Gyro(Protocol):
    def calibrate(self) -> None: ...
    def reset(self, heading_deg: float = 0.0) -> None: ...
    def read(self) -> GyroReading: ...


class SimulatedGyro:
    """Simple in-memory gyro for development without hardware."""

    def __init__(self):
        self.heading_deg = 0.0
        self.angular_rate_dps = 0.0
        self.calibrated = False

    def calibrate(self) -> None:
        self.calibrated = True

    def reset(self, heading_deg: float = 0.0) -> None:
        self.heading_deg = heading_deg
        self.angular_rate_dps = 0.0

    def set_heading(self, heading_deg: float, angular_rate_dps: float = 0.0) -> None:
        self.heading_deg = heading_deg
        self.angular_rate_dps = angular_rate_dps

    def read(self) -> GyroReading:
        return GyroReading(
            heading_deg=self.heading_deg,
            angular_rate_dps=self.angular_rate_dps,
            calibrated=self.calibrated,
        )


class HardwareGyro:
    """Placeholder for the real gyro implementation.

    Once the hardware is known, this class can be updated with the right bus,
    driver, and calibration routine.
    """

    def __init__(self):
        self._heading_deg = 0.0
        self._calibrated = False

    def calibrate(self) -> None:
        self._calibrated = True

    def reset(self, heading_deg: float = 0.0) -> None:
        self._heading_deg = heading_deg

    def read(self) -> GyroReading:
        return GyroReading(
            heading_deg=self._heading_deg,
            angular_rate_dps=0.0,
            calibrated=self._calibrated,
        )


def create_gyro(simulate: bool = True) -> Gyro:
    if simulate:
        gyro = SimulatedGyro()
    else:
        gyro = HardwareGyro()
    gyro.calibrate()
    return gyro
