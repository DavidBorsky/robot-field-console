"""Path following logic for the robot-side autonomy stack.

This module sits between saved waypoints and low-level drivetrain commands.
It now uses a more explicit Pure Pursuit lookahead calculation by intersecting
the robot's lookahead circle with path segments, and it leaves room for future
controllers such as Ramsete.
"""

from math import acos, sqrt
from typing import Dict, List, Optional, Set, Tuple

try:
    from typing import Protocol
except ImportError:  # pragma: no cover - Python < 3.8 fallback
    class Protocol(object):
        pass

from constants import (
    DEFAULT_CONTROL_DT_S,
    PLAYABLE_MAX_X_IN,
    PLAYABLE_MAX_Y_IN,
    PLAYABLE_MIN_X_IN,
    PLAYABLE_MIN_Y_IN,
    FOLLOWER_ACCEL_RAMP_PER_S,
    FOLLOWER_CORNER_SLOWDOWN_GAIN,
    FOLLOWER_CORNER_STOP_DISTANCE_IN,
    FOLLOWER_CORNER_STOP_DURATION_S,
    FOLLOWER_STOP_BRAKE_DISTANCE_IN,
    FOLLOWER_DECEL_RAMP_PER_S,
    FOLLOWER_LONG_RUN_DISTANCE_IN,
    FOLLOWER_SLOWDOWN_DISTANCE_IN,
    MAX_DRIVE_OUTPUT,
)
from drivetrain import MotorCommand, PathFollower as DriveController
from odom import Pose2D


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class PathPoint:
    def __init__(self, x, y):
        self.x = clamp(x, PLAYABLE_MIN_X_IN, PLAYABLE_MAX_X_IN)
        self.y = clamp(y, PLAYABLE_MIN_Y_IN, PLAYABLE_MAX_Y_IN)


class ClosestPathSample:
    def __init__(self, point, segment_index, t, distance):
        self.point = point
        self.segment_index = segment_index
        self.t = t
        self.distance = distance


class FollowerController(Protocol):
    def load_path(self, points: List[PathPoint]) -> None: ...
    def reset(self) -> None: ...
    def is_finished(self, pose: Pose2D) -> bool: ...
    def update(self, pose: Pose2D, dt: float = DEFAULT_CONTROL_DT_S) -> Tuple[MotorCommand, Dict[str, float]]: ...


class PurePursuitFollower:
    """Pure Pursuit controller for sampled waypoint paths."""

    def __init__(
        self,
        lookahead_in: float = 3.0,
        min_speed: float = 0.1,
        max_speed: float = 1.0,
        goal_tolerance_in: float = 0.75,
        accel_ramp_per_s: float = FOLLOWER_ACCEL_RAMP_PER_S,
        decel_ramp_per_s: float = FOLLOWER_DECEL_RAMP_PER_S,
        long_run_distance_in: float = FOLLOWER_LONG_RUN_DISTANCE_IN,
        slowdown_distance_in: float = FOLLOWER_SLOWDOWN_DISTANCE_IN,
        corner_slowdown_gain: float = FOLLOWER_CORNER_SLOWDOWN_GAIN,
        corner_stop_duration_s: float = FOLLOWER_CORNER_STOP_DURATION_S,
        corner_stop_distance_in: float = FOLLOWER_CORNER_STOP_DISTANCE_IN,
        stop_brake_distance_in: float = FOLLOWER_STOP_BRAKE_DISTANCE_IN,
    ):
        self.lookahead_in = lookahead_in
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.goal_tolerance_in = goal_tolerance_in
        self.accel_ramp_per_s = accel_ramp_per_s
        self.decel_ramp_per_s = decel_ramp_per_s
        self.long_run_distance_in = long_run_distance_in
        self.slowdown_distance_in = slowdown_distance_in
        self.corner_slowdown_gain = corner_slowdown_gain
        self.corner_stop_duration_s = corner_stop_duration_s
        self.corner_stop_distance_in = corner_stop_distance_in
        self.stop_brake_distance_in = stop_brake_distance_in
        self.drive_controller = DriveController()
        self.path = []  # type: List[PathPoint]
        self.current_index = 0
        self.last_speed_scale = min_speed
        self.corner_stop_remaining_s = 0.0
        self.completed_corner_stops = set()  # type: Set[int]

    def load_path(self, points: List[PathPoint]) -> None:
        self.path = points
        self.reset()

    def reset(self) -> None:
        self.current_index = 0
        self.last_speed_scale = self.min_speed
        self.corner_stop_remaining_s = 0.0
        self.completed_corner_stops.clear()
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

        best = None  # type: Optional[ClosestPathSample]
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
    ) -> List[Tuple[float, PathPoint]]:
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
        hits = []  # type: List[Tuple[float, PathPoint]]
        for sign in (-1.0, 1.0):
            t = (-b + sign * root) / (2.0 * a)
            if 0.0 <= t <= 1.0:
                hits.append((t, PathPoint(x=start.x + t * dx, y=start.y + t * dy)))
        return hits

    def _select_lookahead_point(self, pose: Pose2D) -> Tuple[PathPoint, ClosestPathSample]:
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

    def _segment_turn_severity(self, closest: ClosestPathSample) -> float:
        if len(self.path) < 3:
            return 0.0

        segment_index = min(max(closest.segment_index, 0), len(self.path) - 2)
        if segment_index >= len(self.path) - 2:
            return 0.0

        first_start = self.path[segment_index]
        first_end = self.path[segment_index + 1]
        second_end = self.path[segment_index + 2]

        ax = first_end.x - first_start.x
        ay = first_end.y - first_start.y
        bx = second_end.x - first_end.x
        by = second_end.y - first_end.y

        a_mag = sqrt(ax * ax + ay * ay)
        b_mag = sqrt(bx * bx + by * by)
        if a_mag <= 1e-9 or b_mag <= 1e-9:
            return 0.0

        dot = ax * bx + ay * by
        cos_theta = clamp(dot / (a_mag * b_mag), -1.0, 1.0)
        turn_angle = acos(cos_theta)
        return clamp(turn_angle / 3.141592653589793, 0.0, 1.0)

    def _distance_until_turn(self, closest: ClosestPathSample) -> float:
        if len(self.path) < 3:
            return float("inf")

        segment_index = min(max(closest.segment_index, 0), len(self.path) - 2)
        if segment_index >= len(self.path) - 2:
            return float("inf")

        current_end = self.path[segment_index + 1]
        dx = current_end.x - closest.point.x
        dy = current_end.y - closest.point.y
        return sqrt(dx * dx + dy * dy)

    def _distance_until_stop(self, closest: ClosestPathSample, pose: Pose2D) -> float:
        distance_until_turn = self._distance_until_turn(closest)
        remaining = pose.distance_to(self.path[-1].x, self.path[-1].y)
        return min(distance_until_turn, remaining)

    def _compute_speed_scale(
        self,
        pose: Pose2D,
        target: PathPoint,
        closest: ClosestPathSample,
        dt: float,
    ) -> Tuple[float, Dict[str, float]]:
        remaining = pose.distance_to(self.path[-1].x, self.path[-1].y)
        target_dist = pose.distance_to(target.x, target.y)
        turn_severity = self._segment_turn_severity(closest)
        distance_until_turn = self._distance_until_turn(closest)
        distance_until_stop = self._distance_until_stop(closest, pose)
        long_run_scale = clamp(
            remaining / max(self.long_run_distance_in, 1e-6),
            self.min_speed,
            self.max_speed,
        )

        # Speed up on long straight runs, then brake for the end of the path,
        # nearby corners, and very close lookahead points.
        end_scale = clamp(
            remaining / max(self.slowdown_distance_in, 1e-6),
            self.min_speed,
            self.max_speed,
        )
        normalized_stop_distance = clamp(
            distance_until_stop / max(self.stop_brake_distance_in, 1e-6),
            0.0,
            1.0,
        )
        stop_scale = clamp(normalized_stop_distance * normalized_stop_distance, 0.02, self.max_speed)
        target_scale = clamp(target_dist / max(self.lookahead_in, 1e-6), self.min_speed, 1.0)
        corner_scale = self.max_speed
        if turn_severity > 0.0 and distance_until_turn != float("inf"):
            corner_window = max(self.lookahead_in * 2.0, 1e-6)
            approach_scale = clamp(distance_until_turn / corner_window, 0.0, 1.0)
            approach_weight = approach_scale * approach_scale
            raw_corner_scale = self.max_speed - self.corner_slowdown_gain * turn_severity
            corner_floor = clamp(raw_corner_scale, 0.12, self.max_speed)
            corner_scale = clamp(
                corner_floor + (self.max_speed - corner_floor) * approach_weight,
                self.min_speed,
                self.max_speed,
            )
        requested_scale = clamp(
            min(long_run_scale, end_scale, stop_scale, target_scale, corner_scale),
            self.min_speed,
            self.max_speed,
        )
        ramp_rate = self.accel_ramp_per_s if requested_scale >= self.last_speed_scale else self.decel_ramp_per_s
        max_delta = ramp_rate * max(dt, 1e-6)
        smoothed_scale = clamp(
            requested_scale,
            self.last_speed_scale - max_delta,
            self.last_speed_scale + max_delta,
        )
        self.last_speed_scale = smoothed_scale
        return smoothed_scale, {
            "long_run_scale": long_run_scale,
            "end_scale": end_scale,
            "target_scale": target_scale,
            "stop_scale": stop_scale,
            "corner_scale": corner_scale,
            "distance_until_turn": distance_until_turn,
            "distance_until_stop": distance_until_stop,
        }

    def update(
        self,
        pose: Pose2D,
        dt: float = DEFAULT_CONTROL_DT_S,
    ) -> Tuple[MotorCommand, Dict[str, float]]:
        if not self.path:
            return MotorCommand(0.0, 0.0), {"finished": True, "reason": "empty path"}

        if self.corner_stop_remaining_s > 0.0:
            self.corner_stop_remaining_s = max(0.0, self.corner_stop_remaining_s - dt)
            return MotorCommand(0.0, 0.0), {
                "finished": False,
                "reason": "corner_stop",
                "paused_for_corner": True,
                "corner_stop_remaining_s": self.corner_stop_remaining_s,
                "speed_scale": 0.0,
                "turn_severity": 0.0,
                "remaining_distance": pose.distance_to(self.path[-1].x, self.path[-1].y),
                "current_index": self.current_index,
                "lookahead_x": pose.x,
                "lookahead_y": pose.y,
            }

        lookahead, closest = self._select_lookahead_point(pose)
        turn_severity = self._segment_turn_severity(closest)
        distance_until_turn = self._distance_until_turn(closest)
        if (
            turn_severity > 0.08
            and distance_until_turn <= self.corner_stop_distance_in
            and closest.segment_index not in self.completed_corner_stops
        ):
            self.completed_corner_stops.add(closest.segment_index)
            self.corner_stop_remaining_s = self.corner_stop_duration_s
            return MotorCommand(0.0, 0.0), {
                "finished": False,
                "reason": "corner_stop",
                "paused_for_corner": True,
                "corner_stop_remaining_s": self.corner_stop_remaining_s,
                "speed_scale": 0.0,
                "turn_severity": turn_severity,
                "remaining_distance": pose.distance_to(self.path[-1].x, self.path[-1].y),
                "current_index": self.current_index,
                "lookahead_x": lookahead.x,
                "lookahead_y": lookahead.y,
                "distance_until_turn": distance_until_turn,
            }

        speed_scale, speed_debug = self._compute_speed_scale(pose, lookahead, closest, dt)

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
                "turn_severity": turn_severity,
                "remaining_distance": pose.distance_to(goal.x, goal.y),
                "current_index": self.current_index,
                "paused_for_corner": False,
                "corner_stop_remaining_s": 0.0,
            }
        )
        debug.update(speed_debug)
        return scaled_command, debug


class RamseteFollower:
    """Reserved slot for a future Ramsete controller.

    Ramsete will make the most sense once:
    - real encoder velocity feedback exists
    - drivetrain kinematics are finalized
    - heading/pose estimation is tighter
    """

    def __init__(self):
        self.path = []  # type: List[PathPoint]

    def load_path(self, points: List[PathPoint]) -> None:
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
    ) -> Tuple[MotorCommand, Dict[str, float]]:
        raise NotImplementedError(
            "RamseteFollower is intentionally reserved for later once hardware "
            "kinematics and velocity feedback are finalized."
        )


def create_follower(controller: str = "pure_pursuit") -> FollowerController:
    if controller == "pure_pursuit":
        return PurePursuitFollower()
    if controller == "ramsete":
        return RamseteFollower()
    raise ValueError("Unknown follower controller: {}".format(controller))
