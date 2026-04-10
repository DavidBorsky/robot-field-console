"""Reusable PID helpers for drivetrain and heading control."""

from __future__ import annotations

from dataclasses import dataclass
from math import pi

from constants import MAX_INTEGRAL


def wrap_angle_radians(angle: float) -> float:
    """Wrap an angle into the [-pi, pi) range."""
    while angle >= pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


@dataclass
class PIDConfig:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    output_min: float | None = None
    output_max: float | None = None
    integral_limit: float | None = MAX_INTEGRAL
    continuous: bool = False


class PIDController:
    """Small PID controller with optional wraparound error support."""

    def __init__(self, config: PIDConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error: float | None = None

    def _normalize_error(self, error: float) -> float:
        if self.config.continuous:
            return wrap_angle_radians(error)
        return error

    def update(self, target: float, measurement: float, dt: float) -> float:
        if dt <= 0:
            raise ValueError("dt must be positive")

        error = self._normalize_error(target - measurement)
        self.integral += error * dt

        if self.config.integral_limit is not None:
            limit = abs(self.config.integral_limit)
            self.integral = max(-limit, min(limit, self.integral))

        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt

        output = (
            self.config.kp * error
            + self.config.ki * self.integral
            + self.config.kd * derivative
        )

        self.previous_error = error

        if self.config.output_min is not None:
            output = max(self.config.output_min, output)
        if self.config.output_max is not None:
            output = min(self.config.output_max, output)
        return output
