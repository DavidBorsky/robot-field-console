"""Odometry helpers for the current robot concepts."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, radians, sin

from constants import TRACK_WIDTH_IN


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


class DifferentialOdometry:
    """Tracks a 2D robot pose from wheel motion and optional gyro heading."""

    def __init__(self, track_width_in: float = TRACK_WIDTH_IN):
        if track_width_in <= 0:
            raise ValueError("track_width_in must be positive")
        self.track_width_in = track_width_in
        self.pose = Pose2D()

    def reset(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 0.0) -> None:
        self.pose = Pose2D(x=x, y=y, heading_rad=radians(heading_deg))

    def update(
        self,
        left_distance_in: float,
        right_distance_in: float,
        heading_deg: float | None = None,
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
            new_heading = self.pose.heading_rad + delta_heading
        else:
            new_heading = radians(heading_deg)

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

    def __init__(self):
        self.pose = Pose2D()

    def reset(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 0.0) -> None:
        self.pose = Pose2D(x=x, y=y, heading_rad=radians(heading_deg))

    def update(
        self,
        front_distance_in: float,
        back_distance_in: float,
        heading_deg: float | None = None,
    ) -> Pose2D:
        robot_forward = (front_distance_in + back_distance_in) / 2.0
        robot_strafe = (front_distance_in - back_distance_in) / 2.0

        if heading_deg is not None:
            self.pose.heading_rad = radians(heading_deg)

        heading = self.pose.heading_rad

        # Rotate robot-frame translation into field-frame translation.
        self.pose.x += robot_forward * cos(heading) - robot_strafe * sin(heading)
        self.pose.y += robot_forward * sin(heading) + robot_strafe * cos(heading)
        return self.pose

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
