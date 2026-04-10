"""Custom drivetrain helpers for a two-motor front/back mecanum layout."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin

from constants import (
    DEFAULT_CONTROL_DT_S,
    DRIVE_KD,
    DRIVE_KI,
    DRIVE_KP,
    MAX_DRIVE_OUTPUT,
    MAX_TURN_OUTPUT,
    TURN_KD,
    TURN_KI,
    TURN_KP,
)
from odom import Pose2D
from pid import PIDConfig, PIDController, wrap_angle_radians


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class MotorCommand:
    front_output: float
    back_output: float


class FrontBackDrive:
    """Maps robot-frame motion requests into front/back motor outputs.

    For the current planned chassis:
    - equal front/back output produces forward/back motion
    - opposite front/back output produces left/right strafe
    """

    def mix(self, forward: float, strafe: float) -> MotorCommand:
        front = clamp(forward + strafe, -MAX_DRIVE_OUTPUT, MAX_DRIVE_OUTPUT)
        back = clamp(forward - strafe, -MAX_DRIVE_OUTPUT, MAX_DRIVE_OUTPUT)
        return MotorCommand(front_output=front, back_output=back)

    def stop(self) -> MotorCommand:
        return MotorCommand(front_output=0.0, back_output=0.0)


class PathFollower:
    """High-level path helper that converts waypoint error into motor outputs."""

    def __init__(self):
        self.distance_pid = PIDController(
            PIDConfig(
                kp=DRIVE_KP,
                ki=DRIVE_KI,
                kd=DRIVE_KD,
                output_min=-MAX_DRIVE_OUTPUT,
                output_max=MAX_DRIVE_OUTPUT,
            )
        )
        self.strafe_pid = PIDController(
            PIDConfig(
                kp=DRIVE_KP,
                ki=DRIVE_KI,
                kd=DRIVE_KD,
                output_min=-MAX_DRIVE_OUTPUT,
                output_max=MAX_DRIVE_OUTPUT,
            )
        )
        self.heading_pid = PIDController(
            PIDConfig(
                kp=TURN_KP,
                ki=TURN_KI,
                kd=TURN_KD,
                output_min=-MAX_TURN_OUTPUT,
                output_max=MAX_TURN_OUTPUT,
                continuous=True,
            )
        )
        self.drive = FrontBackDrive()

    def reset(self) -> None:
        self.distance_pid.reset()
        self.strafe_pid.reset()
        self.heading_pid.reset()

    def command_to_waypoint(
        self,
        pose: Pose2D,
        target_x: float,
        target_y: float,
        target_heading_rad: float | None = None,
        dt: float = DEFAULT_CONTROL_DT_S,
    ) -> tuple[MotorCommand, dict]:
        dx = target_x - pose.x
        dy = target_y - pose.y

        # Convert field-frame position error into robot-frame forward/strafe error.
        robot_forward_error = dx * cos(pose.heading_rad) + dy * sin(pose.heading_rad)
        robot_strafe_error = -dx * sin(pose.heading_rad) + dy * cos(pose.heading_rad)

        forward_cmd = self.distance_pid.update(robot_forward_error, 0.0, dt)
        strafe_cmd = self.strafe_pid.update(robot_strafe_error, 0.0, dt)

        heading_error = 0.0
        heading_cmd = 0.0
        if target_heading_rad is not None:
            heading_error = wrap_angle_radians(target_heading_rad - pose.heading_rad)
            heading_cmd = self.heading_pid.update(target_heading_rad, pose.heading_rad, dt)

        # The current two-motor layout cannot independently rotate in this model,
        # so heading_cmd is reported for diagnostics/tuning and future hardware.
        command = self.drive.mix(forward=forward_cmd, strafe=strafe_cmd)
        debug = {
            "dx": dx,
            "dy": dy,
            "robot_forward_error": robot_forward_error,
            "robot_strafe_error": robot_strafe_error,
            "heading_error_rad": heading_error,
            "heading_command": heading_cmd,
        }
        return command, debug
