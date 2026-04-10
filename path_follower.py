"""Path following logic for the robot-side autonomy stack.

This module sits between saved waypoints and low-level drivetrain commands.
It now uses a more explicit Pure Pursuit lookahead calculation by intersecting
the robot's lookahead circle with path segments, and it leaves room for future
controllers such as Ramsete.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Protocol

from constants import DEFAULT_CONTROL_DT_S, MAX_DRIVE_OUTPUT
from drivetrain import MotorCommand, PathFollower as DriveController
from odom import Pose2D


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float


@dataclass(frozen=True)
class ClosestPathSample:
    point: PathPoint
    segment_index: int
    t: float
    distance: float


class FollowerController(Protocol):
    def load_path(self, points: list[PathPoint]) -> None: ...
    def reset(self) -> None: ...
    def is_finished(self, pose: Pose2D) -> bool: ...
    def update(self, pose: Pose2D, dt: float = DEFAULT_CONTROL_DT_S) -> tuple[MotorCommand, dict]: ...


class PurePursuitFollower:
    """Pure Pursuit controller for sampled waypoint paths."""

    def __init__(
        self,
        lookahead_in: float = 3.0,
        min_speed: float = 0.1,
        max_speed: float = 1.0,
        goal_tolerance_in: float = 0.75,
    ):
        self.lookahead_in = lookahead_in
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.goal_tolerance_in = goal_tolerance_in
        self.drive_controller = DriveController()
        self.path: list[PathPoint] = []
        self.current_index = 0

    def load_path(self, points: list[PathPoint]) -> None:
        self.path = points
        self.reset()

    def reset(self) -> None:
        self.current_index = 0
        self.drive_controller.reset()

    def is_finished(self, pose: Pose2D) -> bool:
        if not self.path:
            return True
        goal = self.path[-1]
        return (
            self.current_index >= len(self.path) - 1
            and pose.distance_to(goal.x, goal.y) <= self.goal_tolerance_in
        )

    def _closest_point_on_segment(
        self,
        pose: Pose2D,
        start: PathPoint,
        end: PathPoint,
        segment_index: int,
    ) -> ClosestPathSample:
        vx = end.x - start.x
        vy = end.y - start.y
        seg_len_sq = vx * vx + vy * vy

        if seg_len_sq <= 1e-9:
            dist = pose.distance_to(start.x, start.y)
            return ClosestPathSample(point=start, segment_index=segment_index, t=0.0, distance=dist)

        wx = pose.x - start.x
        wy = pose.y - start.y
        t = clamp((wx * vx + wy * vy) / seg_len_sq, 0.0, 1.0)
        proj = PathPoint(x=start.x + t * vx, y=start.y + t * vy)
        return ClosestPathSample(
            point=proj,
            segment_index=segment_index,
            t=t,
            distance=pose.distance_to(proj.x, proj.y),
        )

    def _find_closest_path_sample(self, pose: Pose2D) -> ClosestPathSample:
        if len(self.path) == 1:
            only = self.path[0]
            return ClosestPathSample(
                point=only,
                segment_index=0,
                t=0.0,
                distance=pose.distance_to(only.x, only.y),
            )

        best: ClosestPathSample | None = None
        for i in range(len(self.path) - 1):
            sample = self._closest_point_on_segment(pose, self.path[i], self.path[i + 1], i)
            if best is None or sample.distance < best.distance:
                best = sample

        if best is None:
            raise ValueError("Could not find closest path sample")
        return best

    def _advance_index(self, closest: ClosestPathSample) -> None:
        self.current_index = max(self.current_index, closest.segment_index)

    def _segment_circle_intersection(
        self,
        pose: Pose2D,
        start: PathPoint,
        end: PathPoint,
    ) -> list[tuple[float, PathPoint]]:
        dx = end.x - start.x
        dy = end.y - start.y
        fx = start.x - pose.x
        fy = start.y - pose.y

        a = dx * dx + dy * dy
        if a <= 1e-9:
            return []

        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - self.lookahead_in * self.lookahead_in
        discriminant = b * b - 4.0 * a * c

        if discriminant < 0:
            return []

        root = sqrt(discriminant)
        hits: list[tuple[float, PathPoint]] = []
        for sign in (-1.0, 1.0):
            t = (-b + sign * root) / (2.0 * a)
            if 0.0 <= t <= 1.0:
                hits.append((t, PathPoint(x=start.x + t * dx, y=start.y + t * dy)))
        return hits

    def _select_lookahead_point(self, pose: Pose2D) -> tuple[PathPoint, ClosestPathSample]:
        if not self.path:
            raise ValueError("Cannot select a lookahead point from an empty path")

        closest = self._find_closest_path_sample(pose)
        self._advance_index(closest)

        if len(self.path) == 1:
            return self.path[0], closest

        for i in range(closest.segment_index, len(self.path) - 1):
            intersections = self._segment_circle_intersection(pose, self.path[i], self.path[i + 1])
            if not intersections:
                continue

            # Prefer the furthest-forward intersection on the first segment that hits.
            best_t, best_point = max(intersections, key=lambda item: item[0])
            _ = best_t
            return best_point, closest

        return self.path[-1], closest

    def _compute_speed_scale(self, pose: Pose2D, target: PathPoint) -> float:
        remaining = pose.distance_to(self.path[-1].x, self.path[-1].y)
        target_dist = pose.distance_to(target.x, target.y)

        # Slow near the end of the path, but keep a minimum crawl speed.
        end_scale = clamp(remaining / 12.0, self.min_speed, self.max_speed)
        target_scale = clamp(target_dist / max(self.lookahead_in, 1e-6), self.min_speed, 1.0)
        return clamp(min(end_scale, target_scale), self.min_speed, self.max_speed)

    def update(
        self,
        pose: Pose2D,
        dt: float = DEFAULT_CONTROL_DT_S,
    ) -> tuple[MotorCommand, dict]:
        if not self.path:
            return MotorCommand(0.0, 0.0), {"finished": True, "reason": "empty path"}

        lookahead, closest = self._select_lookahead_point(pose)
        speed_scale = self._compute_speed_scale(pose, lookahead)

        command, debug = self.drive_controller.command_to_waypoint(
            pose=pose,
            target_x=lookahead.x,
            target_y=lookahead.y,
            dt=dt,
        )

        scaled_command = MotorCommand(
            front_output=clamp(command.front_output * speed_scale, -MAX_DRIVE_OUTPUT, MAX_DRIVE_OUTPUT),
            back_output=clamp(command.back_output * speed_scale, -MAX_DRIVE_OUTPUT, MAX_DRIVE_OUTPUT),
        )

        goal = self.path[-1]
        debug.update(
            {
                "finished": self.is_finished(pose),
                "lookahead_x": lookahead.x,
                "lookahead_y": lookahead.y,
                "closest_x": closest.point.x,
                "closest_y": closest.point.y,
                "closest_distance": closest.distance,
                "speed_scale": speed_scale,
                "remaining_distance": pose.distance_to(goal.x, goal.y),
                "current_index": self.current_index,
            }
        )
        return scaled_command, debug


class RamseteFollower:
    """Reserved slot for a future Ramsete controller.

    Ramsete will make the most sense once:
    - real encoder velocity feedback exists
    - drivetrain kinematics are finalized
    - heading/pose estimation is tighter
    """

    def __init__(self):
        self.path: list[PathPoint] = []

    def load_path(self, points: list[PathPoint]) -> None:
        self.path = points

    def reset(self) -> None:
        return None

    def is_finished(self, pose: Pose2D) -> bool:
        if not self.path:
            return True
        goal = self.path[-1]
        return pose.distance_to(goal.x, goal.y) <= 0.75

    def update(
        self,
        pose: Pose2D,
        dt: float = DEFAULT_CONTROL_DT_S,
    ) -> tuple[MotorCommand, dict]:
        raise NotImplementedError(
            "RamseteFollower is intentionally reserved for later once hardware "
            "kinematics and velocity feedback are finalized."
        )


def create_follower(controller: str = "pure_pursuit") -> FollowerController:
    if controller == "pure_pursuit":
        return PurePursuitFollower()
    if controller == "ramsete":
        return RamseteFollower()
    raise ValueError(f"Unknown follower controller: {controller}")
