"""Path following logic for the robot-side autonomy stack."""

from math import acos, copysign, sqrt
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
    """Axis-aligned segment follower for a front/back mecanum robot.

    The robot can only move forward/back on X or strafe on Y, so the path is
    executed one routed segment at a time:
    - drive the active segment using odometry projected onto that segment axis
    - slow down as the segment end approaches
    - pause briefly at corners
    - then advance to the next segment
    """

    ACTIVE_AXIS_MIN_OUTPUT = 0.22

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
        segment_arrival_tolerance_in: float = 0.35,
    ):
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
        self.segment_arrival_tolerance_in = segment_arrival_tolerance_in
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
        if len(self.path) < 2:
            return True
        return self.current_index >= len(self.path) - 1

    def _active_segment_points(self) -> Tuple[PathPoint, PathPoint]:
        if len(self.path) < 2:
            only = self.path[0]
            return only, only
        active_segment_index = min(max(self.current_index, 0), len(self.path) - 2)
        return self.path[active_segment_index], self.path[active_segment_index + 1]

    def _project_pose_to_active_segment_axis(self, pose: Pose2D) -> Pose2D:
        if len(self.path) < 2:
            return Pose2D(x=pose.x, y=pose.y, heading_rad=pose.heading_rad)
        start, end = self._active_segment_points()
        if abs(end.x - start.x) >= abs(end.y - start.y):
            return Pose2D(x=pose.x, y=start.y, heading_rad=pose.heading_rad)
        return Pose2D(x=start.x, y=pose.y, heading_rad=pose.heading_rad)

    def _remaining_on_active_segment(self, pose: Pose2D) -> float:
        start, end = self._active_segment_points()
        projected_pose = self._project_pose_to_active_segment_axis(pose)
        if abs(end.x - start.x) >= abs(end.y - start.y):
            return abs(end.x - projected_pose.x)
        return abs(end.y - projected_pose.y)

    def _distance_to_goal_on_axis(self, pose: Pose2D) -> float:
        projected_pose = self._project_pose_to_active_segment_axis(pose)
        goal = self.path[-1]
        return projected_pose.distance_to(goal.x, goal.y)

    def _projected_debug_fields(self, pose: Pose2D) -> Dict[str, float]:
        projected_pose = self._project_pose_to_active_segment_axis(pose)
        return {
            "projected_pose_x": projected_pose.x,
            "projected_pose_y": projected_pose.y,
        }

    def _segment_turn_severity(self, closest: ClosestPathSample) -> float:
        if len(self.path) < 3 or self.current_index >= len(self.path) - 2:
            return 0.0
        segment_index = self.current_index
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

    def _distance_until_turn(self, pose: Pose2D) -> float:
        if len(self.path) < 3 or self.current_index >= len(self.path) - 2:
            return float("inf")
        return self._remaining_on_active_segment(pose)

    def _compute_speed_scale(
        self,
        pose: Pose2D,
        dt: float,
    ) -> Tuple[float, Dict[str, float]]:
        remaining = self._distance_to_goal_on_axis(pose)
        distance_until_turn = self._remaining_on_active_segment(pose)
        distance_until_stop = min(distance_until_turn, remaining)
        turn_severity = self._segment_turn_severity(ClosestPathSample(point=self.path[self.current_index], segment_index=self.current_index, t=0.0, distance=0.0))
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
        corner_scale = self.max_speed
        if turn_severity > 0.0 and distance_until_turn != float("inf"):
            corner_window = max(self.stop_brake_distance_in, 1e-6)
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
            min(long_run_scale, end_scale, stop_scale, corner_scale),
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
            "stop_scale": stop_scale,
            "corner_scale": corner_scale,
            "distance_until_turn": distance_until_turn,
            "distance_until_stop": distance_until_stop,
        }

    def _apply_active_axis_output_floor(
        self,
        command: MotorCommand,
        segment_start: PathPoint,
        segment_end: PathPoint,
    ) -> MotorCommand:
        if abs(segment_end.x - segment_start.x) >= abs(segment_end.y - segment_start.y):
            sign = 1.0 if command.front_output >= 0.0 else -1.0
            magnitude = max(abs(command.front_output), abs(command.back_output), self.ACTIVE_AXIS_MIN_OUTPUT)
            return MotorCommand(front_output=sign * magnitude, back_output=sign * magnitude)

        sign = 1.0 if (command.front_output - command.back_output) >= 0.0 else -1.0
        magnitude = max(abs(command.front_output), abs(command.back_output), self.ACTIVE_AXIS_MIN_OUTPUT)
        return MotorCommand(front_output=sign * magnitude, back_output=-sign * magnitude)

    def update(
        self,
        pose: Pose2D,
        dt: float = DEFAULT_CONTROL_DT_S,
    ) -> Tuple[MotorCommand, Dict[str, float]]:
        if not self.path:
            return MotorCommand(0.0, 0.0), {
                "finished": True,
                "reason": "empty path",
                "projected_pose_x": pose.x,
                "projected_pose_y": pose.y,
            }

        if self.is_finished(pose):
            goal = self.path[-1]
            return MotorCommand(0.0, 0.0), {
                "finished": True,
                "reason": "complete",
                "paused_for_corner": False,
                "corner_stop_remaining_s": 0.0,
                "speed_scale": 0.0,
                "turn_severity": 0.0,
                "remaining_distance": 0.0,
                "current_index": self.current_index,
                "lookahead_x": goal.x,
                "lookahead_y": goal.y,
                "projected_pose_x": goal.x,
                "projected_pose_y": goal.y,
            }

        if self.corner_stop_remaining_s > 0.0:
            projected_debug = self._projected_debug_fields(pose)
            self.corner_stop_remaining_s = max(0.0, self.corner_stop_remaining_s - dt)
            return MotorCommand(0.0, 0.0), {
                "finished": False,
                "reason": "corner_stop",
                "paused_for_corner": True,
                "corner_stop_remaining_s": self.corner_stop_remaining_s,
                "speed_scale": 0.0,
                "turn_severity": 0.0,
                "remaining_distance": self._distance_to_goal_on_axis(pose),
                "current_index": self.current_index,
                "lookahead_x": pose.x,
                "lookahead_y": pose.y,
                **projected_debug,
            }

        projected_pose = self._project_pose_to_active_segment_axis(pose)
        segment_start, segment_end = self._active_segment_points()
        remaining_on_segment = self._remaining_on_active_segment(projected_pose)
        lookahead = segment_end
        turn_severity = self._segment_turn_severity(
            ClosestPathSample(point=segment_start, segment_index=self.current_index, t=0.0, distance=0.0)
        )
        distance_until_turn = remaining_on_segment

        if remaining_on_segment <= self.segment_arrival_tolerance_in:
            if self.current_index < len(self.path) - 2:
                if self.current_index not in self.completed_corner_stops:
                    self.completed_corner_stops.add(self.current_index)
                    self.corner_stop_remaining_s = self.corner_stop_duration_s
                self.current_index += 1
                next_start, next_end = self._active_segment_points()
                return MotorCommand(0.0, 0.0), {
                    "finished": False,
                    "reason": "corner_stop",
                    "paused_for_corner": True,
                    "corner_stop_remaining_s": self.corner_stop_remaining_s,
                    "speed_scale": 0.0,
                    "turn_severity": turn_severity,
                    "remaining_distance": self._distance_to_goal_on_axis(projected_pose),
                    "current_index": self.current_index,
                    "lookahead_x": next_end.x,
                    "lookahead_y": next_end.y,
                    "distance_until_turn": 0.0,
                    "projected_pose_x": segment_end.x,
                    "projected_pose_y": segment_end.y,
                }

            self.current_index = len(self.path) - 1
            goal = self.path[-1]
            return MotorCommand(0.0, 0.0), {
                "finished": True,
                "reason": "complete",
                "paused_for_corner": False,
                "corner_stop_remaining_s": 0.0,
                "speed_scale": 0.0,
                "turn_severity": 0.0,
                "remaining_distance": 0.0,
                "current_index": self.current_index,
                "lookahead_x": goal.x,
                "lookahead_y": goal.y,
                "distance_until_turn": 0.0,
            }

        if (
            turn_severity > 0.08
            and distance_until_turn <= self.corner_stop_distance_in
            and self.current_index not in self.completed_corner_stops
        ):
            self.completed_corner_stops.add(self.current_index)
            self.corner_stop_remaining_s = self.corner_stop_duration_s
            return MotorCommand(0.0, 0.0), {
                "finished": False,
                "reason": "corner_stop",
                "paused_for_corner": True,
                "corner_stop_remaining_s": self.corner_stop_remaining_s,
                "speed_scale": 0.0,
                "turn_severity": turn_severity,
                "remaining_distance": projected_pose.distance_to(self.path[-1].x, self.path[-1].y),
                "current_index": self.current_index,
                "lookahead_x": lookahead.x,
                "lookahead_y": lookahead.y,
                "distance_until_turn": distance_until_turn,
                "projected_pose_x": projected_pose.x,
                "projected_pose_y": projected_pose.y,
            }

        speed_scale, speed_debug = self._compute_speed_scale(projected_pose, dt)

        command, debug = self.drive_controller.command_to_waypoint(
            pose=projected_pose,
            target_x=lookahead.x,
            target_y=lookahead.y,
            segment_dx=segment_end.x - segment_start.x,
            segment_dy=segment_end.y - segment_start.y,
            dt=dt,
        )

        scaled_command = MotorCommand(
            front_output=clamp(command.front_output * speed_scale, -MAX_DRIVE_OUTPUT, MAX_DRIVE_OUTPUT),
            back_output=clamp(command.back_output * speed_scale, -MAX_DRIVE_OUTPUT, MAX_DRIVE_OUTPUT),
        )

        if not self.is_finished(projected_pose):
            scaled_command = self._apply_active_axis_output_floor(
                scaled_command,
                segment_start,
                segment_end,
            )

        goal = self.path[-1]
        debug.update(
            {
                "finished": False,
                "lookahead_x": lookahead.x,
                "lookahead_y": lookahead.y,
                "closest_x": projected_pose.x,
                "closest_y": projected_pose.y,
                "closest_distance": remaining_on_segment,
                "speed_scale": speed_scale,
                "turn_severity": turn_severity,
                "remaining_distance": projected_pose.distance_to(goal.x, goal.y),
                "current_index": self.current_index,
                "paused_for_corner": False,
                "corner_stop_remaining_s": 0.0,
                "projected_pose_x": projected_pose.x,
                "projected_pose_y": projected_pose.y,
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
