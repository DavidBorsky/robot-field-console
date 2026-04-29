import argparse
import json
import os
from math import atan2, degrees
from pathlib import Path
from threading import Event
from typing import Any, Callable, Dict, List, Optional

from camera import SimulatedCamera, VisionTarget
from connection import create_connection
from constants import (
    DEFAULT_CONTROL_DT_S,
    DEFAULT_POWER_SCALE,
    DEFAULT_SERIAL_PORT_LINUX,
    SIM_MAX_STEPS_PER_RUN,
)
from drivetrain import MotorCommand
from ir_sensor import EdgeSafetyController, IRSensorState
from odom import FrontBackMecanumOdometry
from path_follower import PathPoint, create_follower

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_PATH_FILE = Path("/mnt/c/Users/dbors/Downloads/robot-paths.json")


class Waypoint:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)


def heading_from_points(start_x: float, start_y: float, end_x: float, end_y: float, fallback_deg: float = 0.0) -> float:
    dx = end_x - start_x
    dy = end_y - start_y
    if abs(dx) <= 1e-9 and abs(dy) <= 1e-9:
        return fallback_deg
    return degrees(atan2(dy, dx))


def heading_for_segment(waypoints: List[Waypoint], segment_index: int, fallback_deg: float = 0.0) -> float:
    if len(waypoints) < 2:
        return fallback_deg
    clamped_index = max(0, min(segment_index, len(waypoints) - 2))
    start = waypoints[clamped_index]
    end = waypoints[clamped_index + 1]
    return heading_from_points(start.x, start.y, end.x, end.y, fallback_deg)


def load_robot_paths(path_file: Path = DEFAULT_PATH_FILE) -> dict:
    with path_file.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)

    if "paths" not in payload:
        raise ValueError("robot-paths.json is missing the 'paths' object")

    return payload


def get_waypoints(payload: Dict[str, Any], mode: str = "auton") -> List[Waypoint]:
    raw_points = payload.get("paths", {}).get(mode, [])
    return [Waypoint(float(x), float(y)) for x, y in raw_points]


def emit_status(status_callback: Optional[Callable[[Dict[str, Any]], None]], **payload: Any) -> None:
    if status_callback is None:
        return
    status_callback(payload)


def scale_motor_command(command: MotorCommand, power_scale: float) -> MotorCommand:
    front_output_scale, back_output_scale = get_motor_output_scales()
    return MotorCommand(
        front_output=command.front_output * power_scale * front_output_scale,
        back_output=command.back_output * power_scale * back_output_scale,
    )


def get_open_loop_straight_test_output() -> Optional[float]:
    raw_value = os.environ.get("ROBOT_OPEN_LOOP_STRAIGHT_TEST")
    if raw_value is None or raw_value == "":
        return None
    try:
        output = float(raw_value)
    except ValueError:
        raise ValueError(
            "ROBOT_OPEN_LOOP_STRAIGHT_TEST must be a number between -1.0 and 1.0"
        )
    if output < -1.0 or output > 1.0:
        raise ValueError(
            "ROBOT_OPEN_LOOP_STRAIGHT_TEST must be a number between -1.0 and 1.0"
        )
    return output


def _read_output_scale(name: str) -> float:
    raw_value = os.environ.get(name)
    if raw_value is None or raw_value == "":
        return 1.0
    try:
        scale = float(raw_value)
    except ValueError:
        raise ValueError("{} must be a positive number".format(name))
    if scale <= 0.0:
        raise ValueError("{} must be a positive number".format(name))
    return scale


def get_motor_output_scales() -> tuple:
    return (
        _read_output_scale("ROBOT_FRONT_OUTPUT_SCALE"),
        _read_output_scale("ROBOT_BACK_OUTPUT_SCALE"),
    )


def run_path(
    mode: str = "auton",
    path_file: Path = DEFAULT_PATH_FILE,
    simulate_connection: bool = True,
    follower_controller: str = "pure_pursuit",
    serial_port: str = "/dev/ttyACM0",
    serial_baud: int = 115200,
    power_scale: float = DEFAULT_POWER_SCALE,
    status_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    stop_event: Optional[Event] = None,
    initial_pose: Optional[Dict[str, Any]] = None,
) -> None:
    payload = load_robot_paths(path_file)
    waypoints = get_waypoints(payload, mode)

    if len(waypoints) < 2:
        print(
            "{} path needs at least 2 waypoints before it can run. "
            "Export a path from the UI into {} first.".format(mode, path_file.name)
        )
        emit_status(
            status_callback,
            running=False,
            mode=mode,
            status="idle",
            detail="{} path needs at least 2 waypoints".format(mode),
            pose={"x": 0.0, "y": 0.0, "heading_deg": 0.0},
            velocity_in_per_s=0.0,
            motor_rpm_estimate=0.0,
            velocity_source="encoder",
            step=0,
            max_steps=0,
        )
        return

    print("Loaded {} waypoint(s) for mode '{}'".format(len(waypoints), mode))
    print("Units: {}".format(payload.get("units", "unknown")))
    print("Field: {}".format(payload.get("field", {})))
    print("Follower: {}".format(follower_controller))
    print(
        "Connection mode: {}".format(
            "simulated" if simulate_connection else "serial ({})".format(serial_port)
        )
    )
    print("Power scale: {:.2f}".format(power_scale))
    front_output_scale, back_output_scale = get_motor_output_scales()
    if abs(front_output_scale - 1.0) > 1e-9 or abs(back_output_scale - 1.0) > 1e-9:
        print(
            "Motor output scales: front={:.3f} back={:.3f}".format(
                front_output_scale,
                back_output_scale,
            )
        )

    start_pose = initial_pose or {}
    start_x = float(start_pose.get("x", waypoints[0].x))
    start_y = float(start_pose.get("y", waypoints[0].y))
    start_heading_deg = float(start_pose.get("heading_deg", 0.0))

    odom = FrontBackMecanumOdometry()
    odom.reset(x=start_x, y=start_y, heading_deg=start_heading_deg)
    emit_status(
        status_callback,
        running=True,
        mode=mode,
        status="ready",
        detail="loaded {} waypoint(s)".format(len(waypoints)),
        follower=follower_controller,
        connection="simulated" if simulate_connection else "serial",
        power_scale=power_scale,
        pose={
            "x": odom.pose.x,
            "y": odom.pose.y,
            "heading_deg": odom.pose.heading_deg,
        },
        travel_direction_deg=heading_for_segment(waypoints, 0, 0.0),
        step=0,
        max_steps=SIM_MAX_STEPS_PER_RUN,
        waypoint_count=len(waypoints),
        units=payload.get("units", "unknown"),
        velocity_in_per_s=0.0,
        motor_rpm_estimate=0.0,
        velocity_source="encoder",
    )

    follower = create_follower(follower_controller)
    follower.load_path([PathPoint(x=waypoint.x, y=waypoint.y) for waypoint in waypoints])
    edge_safety = EdgeSafetyController()
    # The Pi HTTP server owns the real USB camera feed.
    # Keep the runner on a simulated vision target path for now so it does not
    # fight the UI camera stream for the device handle during live runs.
    camera = SimulatedCamera()
    connection = create_connection(
        simulate=simulate_connection,
        port=serial_port,
        baudrate=serial_baud,
    )
    camera.start()
    connection.connect()

    for index, waypoint in enumerate(waypoints, start=1):
        print("Waypoint {}: x={:.2f}, y={:.2f}".format(index, waypoint.x, waypoint.y))

    print("\nFollower preview:")
    try:
        open_loop_straight_test_output = get_open_loop_straight_test_output()
        if open_loop_straight_test_output is not None:
            print(
                "Open-loop straight test active: front=back={:.3f}".format(
                    open_loop_straight_test_output * power_scale
                )
            )
        last_command = None
        path_completed = False
        edge_stop_detail = None
        for step in range(1, SIM_MAX_STEPS_PER_RUN + 1):
            if stop_event is not None and stop_event.is_set():
                emit_status(
                    status_callback,
                    running=False,
                    mode=mode,
                    status="stopped",
                    detail="reset requested",
                    pose={
                        "x": odom.pose.x,
                        "y": odom.pose.y,
                        "heading_deg": odom.pose.heading_deg,
                    },
                    step=max(step - 1, 0),
                    max_steps=SIM_MAX_STEPS_PER_RUN,
                    velocity_in_per_s=0.0,
                    motor_rpm_estimate=0.0,
                    velocity_source="encoder",
                )
                break
            sensor_snapshot = connection.read_sensors()
            ir_state = sensor_snapshot.ir

            if simulate_connection and hasattr(connection, "set_sensor_snapshot"):
                # Simulate an edge event once so the protection path can be tested.
                simulated_ir = IRSensorState(left_edge_detected=(step == 12), right_edge_detected=False)
                connection.set_sensor_snapshot(connection.read_sensors().__class__(ir=simulated_ir, heading_deg=None))
                sensor_snapshot = connection.read_sensors()
                ir_state = sensor_snapshot.ir
                if isinstance(camera, SimulatedCamera):
                    camera.set_target(
                        VisionTarget(
                            visible=(step == 18),
                            x_offset_norm=-0.18 if step == 18 else 0.0,
                            y_offset_norm=0.06 if step == 18 else 0.0,
                            confidence=0.85 if step == 18 else 0.0,
                            label="alignment-target" if step == 18 else "",
                        )
                    )

            frame = camera.read_frame()
            vision_target = camera.get_target()

            if ir_state.any_edge_detected:
                odom.apply_edge_correction(
                    left_edge_detected=ir_state.left_edge_detected,
                    right_edge_detected=ir_state.right_edge_detected,
                )

            odom.update_from_encoder_snapshot(
                front_count=sensor_snapshot.encoders.front_count,
                back_count=sensor_snapshot.encoders.back_count,
                front_rpm=sensor_snapshot.encoders.front_rpm,
                back_rpm=sensor_snapshot.encoders.back_rpm,
                dt=DEFAULT_CONTROL_DT_S,
                heading_deg=0.0,
                counts_per_rev=sensor_snapshot.encoders.counts_per_rev,
            )
            requested_command, debug = follower.update(pose=odom.pose)
            travel_direction_deg = heading_from_points(
                odom.pose.x,
                odom.pose.y,
                debug.get("lookahead_x", odom.pose.x),
                debug.get("lookahead_y", odom.pose.y),
                0.0,
            )
            if open_loop_straight_test_output is not None:
                requested_command = MotorCommand(
                    front_output=open_loop_straight_test_output,
                    back_output=open_loop_straight_test_output,
                )
                debug["speed_scale"] = 1.0
                debug["turn_severity"] = 0.0
            edge_correction = edge_safety.apply(requested_command, ir_state)
            command = scale_motor_command(edge_correction.command, power_scale)
            print(
                "- step {}: x={:.2f}, y={:.2f}, front={:.3f}, back={:.3f}, "
                "heading={:.2f}, frame={}, lookahead=({:.2f}, {:.2f}), speed={:.2f}, "
                "turn={:.2f}, remaining={:.2f}, edge_override={}".format(
                    step,
                    odom.pose.x,
                    odom.pose.y,
                    command.front_output,
                    command.back_output,
                    odom.pose.heading_deg,
                    frame.frame_id,
                    debug["lookahead_x"],
                    debug["lookahead_y"],
                    debug["speed_scale"],
                    debug["turn_severity"],
                    debug["remaining_distance"],
                    edge_correction.edge_override_active,
                )
            )
            if debug.get("paused_for_corner"):
                print("  corner stop: waiting {:.2f}s".format(debug.get("corner_stop_remaining_s", 0.0)))
            if edge_correction.edge_override_active:
                print("  safety: {}".format(edge_correction.detail))
            if vision_target.visible:
                print(
                    "  vision: target={} x_offset={:.2f} y_offset={:.2f} confidence={:.2f}".format(
                        vision_target.label,
                        vision_target.x_offset_norm,
                        vision_target.y_offset_norm,
                        vision_target.confidence,
                    )
                )
            if edge_correction.stop_run:
                edge_stop_detail = edge_correction.detail
                connection.send_motor_command(command)
                last_command = command
                emit_status(
                    status_callback,
                    running=False,
                    mode=mode,
                    status="stopped",
                    detail=edge_correction.detail,
                    pose={
                        "x": odom.pose.x,
                        "y": odom.pose.y,
                        "heading_deg": odom.pose.heading_deg,
                    },
                    travel_direction_deg=travel_direction_deg,
                    step=step,
                    max_steps=SIM_MAX_STEPS_PER_RUN,
                    waypoint_index=debug["current_index"],
                    remaining_distance=debug["remaining_distance"],
                    turn_severity=debug["turn_severity"],
                    speed_scale=debug["speed_scale"],
                    velocity_in_per_s=0.0,
                    forward_velocity_in_per_s=0.0,
                    strafe_velocity_in_per_s=0.0,
                    front_encoder_count=sensor_snapshot.encoders.front_count,
                    back_encoder_count=sensor_snapshot.encoders.back_count,
                    front_motor_rpm=0.0,
                    back_motor_rpm=0.0,
                    motor_rpm_estimate=0.0,
                    velocity_source="encoder",
                    power_scale=power_scale,
                    paused_for_corner=debug.get("paused_for_corner", False),
                    corner_stop_remaining_s=debug.get("corner_stop_remaining_s", 0.0),
                    edge_override_active=True,
                    edge_detail=edge_correction.detail,
                    left_edge_detected=ir_state.left_edge_detected,
                    right_edge_detected=ir_state.right_edge_detected,
                    telemetry={
                        "front_motor_temp_c": sensor_snapshot.telemetry.front_motor_temp_c,
                        "back_motor_temp_c": sensor_snapshot.telemetry.back_motor_temp_c,
                        "battery_voltage": sensor_snapshot.telemetry.battery_voltage,
                        "pi_battery_voltage": sensor_snapshot.telemetry.pi_battery_voltage,
                    },
                )
                break
            emit_status(
                status_callback,
                running=True,
                mode=mode,
                status="running",
                detail=(
                    "stopped at field boundary"
                    if debug.get("reason") == "field_boundary_stop"
                    else "edge override active"
                    if edge_correction.edge_override_active
                    else "following path"
                ),
                pose={
                    "x": odom.pose.x,
                    "y": odom.pose.y,
                    "heading_deg": odom.pose.heading_deg,
                },
                travel_direction_deg=travel_direction_deg,
                command={
                    "front_output": command.front_output,
                    "back_output": command.back_output,
                },
                lookahead={
                    "x": debug["lookahead_x"],
                    "y": debug["lookahead_y"],
                },
                step=step,
                max_steps=SIM_MAX_STEPS_PER_RUN,
                waypoint_index=debug["current_index"],
                remaining_distance=debug["remaining_distance"],
                turn_severity=debug["turn_severity"],
                speed_scale=debug["speed_scale"],
                velocity_in_per_s=odom.motion.linear_velocity_in_per_s,
                forward_velocity_in_per_s=odom.motion.robot_forward_velocity_in_per_s,
                strafe_velocity_in_per_s=odom.motion.robot_strafe_velocity_in_per_s,
                front_encoder_count=sensor_snapshot.encoders.front_count,
                back_encoder_count=sensor_snapshot.encoders.back_count,
                front_motor_rpm=sensor_snapshot.encoders.front_rpm,
                back_motor_rpm=sensor_snapshot.encoders.back_rpm,
                motor_rpm_estimate=(
                    abs(sensor_snapshot.encoders.front_rpm) + abs(sensor_snapshot.encoders.back_rpm)
                ) / 2.0,
                velocity_source="encoder",
                power_scale=power_scale,
                paused_for_corner=debug.get("paused_for_corner", False),
                corner_stop_remaining_s=debug.get("corner_stop_remaining_s", 0.0),
                edge_override_active=edge_correction.edge_override_active,
                edge_detail=edge_correction.detail,
                left_edge_detected=ir_state.left_edge_detected,
                right_edge_detected=ir_state.right_edge_detected,
                telemetry={
                    "front_motor_temp_c": sensor_snapshot.telemetry.front_motor_temp_c,
                    "back_motor_temp_c": sensor_snapshot.telemetry.back_motor_temp_c,
                    "battery_voltage": sensor_snapshot.telemetry.battery_voltage,
                    "pi_battery_voltage": sensor_snapshot.telemetry.pi_battery_voltage,
                },
            )
            if stop_event is not None and stop_event.is_set():
                emit_status(
                    status_callback,
                    running=False,
                    mode=mode,
                    status="stopped",
                    detail="reset requested",
                    pose={
                        "x": odom.pose.x,
                        "y": odom.pose.y,
                        "heading_deg": odom.pose.heading_deg,
                    },
                    travel_direction_deg=travel_direction_deg,
                    step=step,
                    max_steps=SIM_MAX_STEPS_PER_RUN,
                    velocity_in_per_s=0.0,
                    motor_rpm_estimate=0.0,
                    velocity_source="encoder",
                )
                break
            connection.send_motor_command(command)
            last_command = command

            if debug.get("reason") == "field_boundary_stop":
                print("  safety: field boundary reached, motors stopped")
                emit_status(
                    status_callback,
                    running=False,
                    mode=mode,
                    status="stopped",
                    detail="field boundary reached",
                    pose={
                        "x": odom.pose.x,
                        "y": odom.pose.y,
                        "heading_deg": odom.pose.heading_deg,
                    },
                    travel_direction_deg=travel_direction_deg,
                    step=step,
                    max_steps=SIM_MAX_STEPS_PER_RUN,
                    waypoint_index=debug["current_index"],
                    remaining_distance=debug["remaining_distance"],
                    velocity_in_per_s=0.0,
                    motor_rpm_estimate=0.0,
                    velocity_source="encoder",
                )
                break

            if debug["finished"]:
                print("  path complete: goal tolerance reached")
                path_completed = True
                emit_status(
                    status_callback,
                    running=False,
                    mode=mode,
                    status="complete",
                    detail="goal tolerance reached",
                    pose={
                        "x": odom.pose.x,
                        "y": odom.pose.y,
                        "heading_deg": odom.pose.heading_deg,
                    },
                    step=step,
                    max_steps=SIM_MAX_STEPS_PER_RUN,
                    waypoint_index=debug["current_index"],
                    remaining_distance=debug["remaining_distance"],
                    velocity_in_per_s=0.0,
                    motor_rpm_estimate=0.0,
                    velocity_source="encoder",
                )
                break

        print(
            "\nNext hardware step: replace the simulated IR events with real "
            "Arduino sensor packets and integrate live vision detections from the Pi camera feed."
        )
    finally:
        connection.stop()
        final_connection_status = connection.status()
        print("Connection status: {}".format(final_connection_status))
        final_status = "idle"
        final_detail = "connection closed: {}".format(final_connection_status.detail)
        if path_completed:
            final_status = "complete"
            final_detail = "run finished and connection closed"
        elif stop_event is not None and stop_event.is_set():
            final_status = "stopped"
            final_detail = "run stopped by reset request"
        elif edge_stop_detail is not None:
            final_status = "stopped"
            final_detail = edge_stop_detail
        elif last_command is not None:
            final_status = "stopped"
            final_detail = "run stopped before reaching the goal"
        emit_status(
            status_callback,
            running=False,
            mode=mode,
            status=final_status,
            detail=final_detail,
            connection=final_connection_status.mode,
            connection_connected=final_connection_status.connected,
            pose={
                "x": odom.pose.x,
                "y": odom.pose.y,
                "heading_deg": odom.pose.heading_deg,
            },
            travel_direction_deg=0.0,
            velocity_in_per_s=0.0,
            motor_rpm_estimate=0.0,
            velocity_source="encoder",
        )
        connection.close()
        camera.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the robot path follower.")
    parser.add_argument("--mode", default="auton", choices=["auton", "teleop"], help="Path mode to run")
    parser.add_argument(
        "--path-file",
        default=str(DEFAULT_PATH_FILE),
        help="Path to robot-paths.json",
    )
    parser.add_argument(
        "--connection",
        default="simulated",
        choices=["simulated", "serial"],
        help="Use simulated output or a real serial port",
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_SERIAL_PORT_LINUX,
        help="Serial port, for example COM3 or /dev/ttyACM0",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--follower",
        default="pure_pursuit",
        choices=["pure_pursuit", "ramsete"],
        help="Follower controller to use",
    )
    args = parser.parse_args()

    run_path(
        mode=args.mode,
        path_file=Path(args.path_file),
        simulate_connection=(args.connection == "simulated"),
        follower_controller=args.follower,
        serial_port=args.port,
        serial_baud=args.baud,
    )
