"""Odometry helpers for the current robot concepts."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, pi, radians, sin
from typing import Optional

from constants import (
    ENCODER_COUNTS_PER_OUTPUT_REV,
    ODOM_FORWARD_SCALE,
    ODOM_STRAFE_SCALE,
    TRACK_WIDTH_IN,
    WHEEL_CIRCUMFERENCE_IN,
)


def wrap_heading_radians(angle: float) -> float:
    """Wrap a heading into the [-pi, pi) range."""
    while angle >= pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    heading_rad: float = 0.0

    @property
    def heading_deg(self) -> float:
        return degrees(self.heading_rad)

    def distance_to(self, x: float, y: float) -> float:
        return hypot(x - self.x, y - self.y)

    def heading_to(self, x: float, y: float) -> float:
        return atan2(y - self.y, x - self.x)


@dataclass
class FrontBackMotionState:
    front_distance_in: float = 0.0
    back_distance_in: float = 0.0
    front_velocity_in_per_s: float = 0.0
    back_velocity_in_per_s: float = 0.0
    robot_forward_velocity_in_per_s: float = 0.0
    robot_strafe_velocity_in_per_s: float = 0.0
    linear_velocity_in_per_s: float = 0.0
    front_rpm: float = 0.0
    back_rpm: float = 0.0


class DifferentialOdometry:
    """Tracks a 2D robot pose from wheel motion and optional gyro heading."""

    def __init__(self, track_width_in: float = TRACK_WIDTH_IN):
        if track_width_in <= 0:
            raise ValueError("track_width_in must be positive")
        self.track_width_in = track_width_in
        self.pose = Pose2D()

    def reset(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 0.0) -> None:
        self.pose = Pose2D(x=x, y=y, heading_rad=wrap_heading_radians(radians(heading_deg)))

    def update(
        self,
        left_distance_in: float,
        right_distance_in: float,
        heading_deg: Optional[float] = None,
    ) -> Pose2D:
        """Apply incremental wheel distances and update the stored pose.

        Args:
            left_distance_in: Incremental left wheel travel since last update.
            right_distance_in: Incremental right wheel travel since last update.
            heading_deg: Optional absolute gyro heading. When provided, gyro
                heading is trusted over wheel-derived heading drift.
        """
        delta_center = (left_distance_in + right_distance_in) / 2.0

        if heading_deg is None:
            delta_heading = (right_distance_in - left_distance_in) / self.track_width_in
            new_heading = wrap_heading_radians(self.pose.heading_rad + delta_heading)
        else:
            new_heading = wrap_heading_radians(radians(heading_deg))

        avg_heading = (self.pose.heading_rad + new_heading) / 2.0
        self.pose.x += delta_center * cos(avg_heading)
        self.pose.y += delta_center * sin(avg_heading)
        self.pose.heading_rad = new_heading
        return self.pose


class FrontBackMecanumOdometry:
    """Pose tracking for a front/back linked mecanum layout.

    This model assumes:
    - equal front/back wheel motion -> forward/back translation
    - opposite front/back wheel motion -> strafe translation
    - heading comes from the gyro when available
    """

    def __init__(
        self,
        forward_scale: float = ODOM_FORWARD_SCALE,
        strafe_scale: float = ODOM_STRAFE_SCALE,
        wheel_circumference_in: float = WHEEL_CIRCUMFERENCE_IN,
        encoder_counts_per_rev: float = ENCODER_COUNTS_PER_OUTPUT_REV,
    ):
        if forward_scale <= 0:
            raise ValueError("forward_scale must be positive")
        if strafe_scale <= 0:
            raise ValueError("strafe_scale must be positive")
        if wheel_circumference_in <= 0:
            raise ValueError("wheel_circumference_in must be positive")
        if encoder_counts_per_rev <= 0:
            raise ValueError("encoder_counts_per_rev must be positive")
        self.pose = Pose2D()
        self.motion = FrontBackMotionState()
        self.forward_scale = forward_scale
        self.strafe_scale = strafe_scale
        self.wheel_circumference_in = wheel_circumference_in
        self.encoder_counts_per_rev = encoder_counts_per_rev
        self._last_front_count = None  # type: Optional[int]
        self._last_back_count = None  # type: Optional[int]

    def reset(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 0.0) -> None:
        self.pose = Pose2D(x=x, y=y, heading_rad=wrap_heading_radians(radians(heading_deg)))
        self.motion = FrontBackMotionState()
        self._last_front_count = None
        self._last_back_count = None

    def _distance_from_counts(self, delta_counts: int, counts_per_rev: float) -> float:
        return (delta_counts / max(counts_per_rev, 1e-6)) * self.wheel_circumference_in

    def _distance_from_rpm(self, rpm: float, dt: float) -> float:
        return (rpm / 60.0) * self.wheel_circumference_in * max(dt, 0.0)

    def _velocity_from_rpm(self, rpm: float) -> float:
        return (rpm / 60.0) * self.wheel_circumference_in

    def update(
        self,
        front_distance_in: float,
        back_distance_in: float,
        heading_deg: Optional[float] = None,
    ) -> Pose2D:
        robot_forward = ((front_distance_in + back_distance_in) / 2.0) * self.forward_scale
        robot_strafe = ((front_distance_in - back_distance_in) / 2.0) * self.strafe_scale

        if heading_deg is not None:
            self.pose.heading_rad = wrap_heading_radians(radians(heading_deg))

        heading = self.pose.heading_rad

        # Rotate robot-frame translation into field-frame translation.
        self.pose.x += robot_forward * cos(heading) - robot_strafe * sin(heading)
        self.pose.y += robot_forward * sin(heading) + robot_strafe * cos(heading)
        self.motion.front_distance_in = front_distance_in
        self.motion.back_distance_in = back_distance_in
        return self.pose

    def update_from_encoder_snapshot(
        self,
        *,
        front_count: int,
        back_count: int,
        front_rpm: Optional[float] = None,
        back_rpm: Optional[float] = None,
        dt: Optional[float] = None,
        heading_deg: Optional[float] = None,
        counts_per_rev: Optional[float] = None,
    ) -> Pose2D:
        counts_per_rev = counts_per_rev or self.encoder_counts_per_rev

        if self._last_front_count is None or self._last_back_count is None:
            self._last_front_count = front_count
            self._last_back_count = back_count
            self.motion.front_rpm = front_rpm or 0.0
            self.motion.back_rpm = back_rpm or 0.0
            self.motion.front_velocity_in_per_s = self._velocity_from_rpm(front_rpm or 0.0)
            self.motion.back_velocity_in_per_s = self._velocity_from_rpm(back_rpm or 0.0)
            self.motion.robot_forward_velocity_in_per_s = (
                self.motion.front_velocity_in_per_s + self.motion.back_velocity_in_per_s
            ) / 2.0
            self.motion.robot_strafe_velocity_in_per_s = (
                self.motion.front_velocity_in_per_s - self.motion.back_velocity_in_per_s
            ) / 2.0
            self.motion.linear_velocity_in_per_s = hypot(
                self.motion.robot_forward_velocity_in_per_s,
                self.motion.robot_strafe_velocity_in_per_s,
            )
            if heading_deg is not None:
                self.pose.heading_rad = wrap_heading_radians(radians(heading_deg))
            return self.pose

        delta_front_counts = front_count - self._last_front_count
        delta_back_counts = back_count - self._last_back_count
        self._last_front_count = front_count
        self._last_back_count = back_count

        if delta_front_counts != 0 or delta_back_counts != 0:
            front_distance_in = self._distance_from_counts(delta_front_counts, counts_per_rev)
            back_distance_in = self._distance_from_counts(delta_back_counts, counts_per_rev)
        elif dt is not None and front_rpm is not None and back_rpm is not None:
            front_distance_in = self._distance_from_rpm(front_rpm, dt)
            back_distance_in = self._distance_from_rpm(back_rpm, dt)
        else:
            front_distance_in = 0.0
            back_distance_in = 0.0

        pose = self.update(
            front_distance_in=front_distance_in,
            back_distance_in=back_distance_in,
            heading_deg=heading_deg,
        )

        self.motion.front_distance_in = front_distance_in
        self.motion.back_distance_in = back_distance_in
        self.motion.front_rpm = front_rpm or 0.0
        self.motion.back_rpm = back_rpm or 0.0
        self.motion.front_velocity_in_per_s = self._velocity_from_rpm(front_rpm or 0.0)
        self.motion.back_velocity_in_per_s = self._velocity_from_rpm(back_rpm or 0.0)
        self.motion.robot_forward_velocity_in_per_s = (
            self.motion.front_velocity_in_per_s + self.motion.back_velocity_in_per_s
        ) / 2.0
        self.motion.robot_strafe_velocity_in_per_s = (
            self.motion.front_velocity_in_per_s - self.motion.back_velocity_in_per_s
        ) / 2.0
        self.motion.linear_velocity_in_per_s = hypot(
            self.motion.robot_forward_velocity_in_per_s,
            self.motion.robot_strafe_velocity_in_per_s,
        )
        return pose

    def apply_edge_correction(
        self,
        *,
        left_edge_detected: bool = False,
        right_edge_detected: bool = False,
        correction_step_in: float = 0.5,
    ) -> Pose2D:
        """Nudge the estimated pose back toward the carpet when an edge is seen.

        This is intentionally simple for now. Once sensor placement is finalized,
        this can become a stronger field-boundary correction.
        """
        if left_edge_detected and not right_edge_detected:
            self.pose.y -= correction_step_in
        elif right_edge_detected and not left_edge_detected:
            self.pose.y += correction_step_in
        elif left_edge_detected and right_edge_detected:
            self.pose.x -= correction_step_in
        return self.pose
