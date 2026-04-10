from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

from camera import SimulatedCamera, VisionTarget, create_camera
from connection import create_connection
from gyro import SimulatedGyro, create_gyro
from ir_sensor import EdgeSafetyController, IRSensorState
from odom import FrontBackMecanumOdometry
from path_follower import PathPoint, create_follower

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_PATH_FILE = Path("/mnt/c/Users/dbors/Downloads/robot-paths.json")


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float


def load_robot_paths(path_file: Path = DEFAULT_PATH_FILE) -> dict:
    with path_file.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)

    if "paths" not in payload:
        raise ValueError("robot-paths.json is missing the 'paths' object")

    return payload


def get_waypoints(payload: dict, mode: str = "auton") -> list[Waypoint]:
    raw_points = payload.get("paths", {}).get(mode, [])
    return [Waypoint(float(x), float(y)) for x, y in raw_points]


def run_path(
    mode: str = "auton",
    path_file: Path = DEFAULT_PATH_FILE,
    simulate_connection: bool = True,
    follower_controller: str = "pure_pursuit",
    serial_port: str = "/dev/ttyACM0",
    serial_baud: int = 115200,
) -> None:
    payload = load_robot_paths(path_file)
    waypoints = get_waypoints(payload, mode)

    if len(waypoints) < 2:
        print(
            f"{mode} path needs at least 2 waypoints before it can run. "
            f"Export a path from the UI into {path_file.name} first."
        )
        return

    print(f"Loaded {len(waypoints)} waypoint(s) for mode '{mode}'")
    print(f"Units: {payload.get('units', 'unknown')}")
    print(f"Field: {payload.get('field', {})}")
    print(f"Follower: {follower_controller}")
    print(f"Connection mode: {'simulated' if simulate_connection else f'serial ({serial_port})'}")

    odom = FrontBackMecanumOdometry()
    odom.reset(x=waypoints[0].x, y=waypoints[0].y, heading_deg=0.0)

    follower = create_follower(follower_controller)
    follower.load_path([PathPoint(x=waypoint.x, y=waypoint.y) for waypoint in waypoints])
    edge_safety = EdgeSafetyController()
    camera = create_camera(simulate=simulate_connection)
    gyro = create_gyro(simulate=simulate_connection)
    connection = create_connection(
        simulate=simulate_connection,
        port=serial_port,
        baudrate=serial_baud,
    )
    camera.start()
    connection.connect()

    for index, waypoint in enumerate(waypoints, start=1):
        print(f"Waypoint {index}: x={waypoint.x:.2f}, y={waypoint.y:.2f}")

    print("\nFollower preview:")
    try:
        for step in range(1, min(len(waypoints) + 2, 6)):
            sensor_snapshot = connection.read_sensors()
            ir_state = sensor_snapshot.ir

            if simulate_connection and hasattr(connection, "set_sensor_snapshot"):
                # Simulate an edge event once so the protection path can be tested.
                simulated_ir = IRSensorState(left_edge_detected=(step == 3), right_edge_detected=False)
                connection.set_sensor_snapshot(connection.read_sensors().__class__(ir=simulated_ir, heading_deg=None))
                sensor_snapshot = connection.read_sensors()
                ir_state = sensor_snapshot.ir
                if isinstance(gyro, SimulatedGyro):
                    gyro.set_heading(heading_deg=(step - 1) * 4.0, angular_rate_dps=4.0)
                if isinstance(camera, SimulatedCamera):
                    camera.set_target(
                        VisionTarget(
                            visible=(step == 4),
                            x_offset_norm=-0.18 if step == 4 else 0.0,
                            y_offset_norm=0.06 if step == 4 else 0.0,
                            confidence=0.85 if step == 4 else 0.0,
                            label="alignment-target" if step == 4 else "",
                        )
                    )

            gyro_reading = gyro.read()
            frame = camera.read_frame()
            vision_target = camera.get_target()

            if ir_state.any_edge_detected:
                odom.apply_edge_correction(
                    left_edge_detected=ir_state.left_edge_detected,
                    right_edge_detected=ir_state.right_edge_detected,
                )

            odom.update(front_distance_in=0.0, back_distance_in=0.0, heading_deg=gyro_reading.heading_deg)
            requested_command, debug = follower.update(pose=odom.pose)
            edge_correction = edge_safety.apply(requested_command, ir_state)
            command = edge_correction.command
            print(
                f"- step {step}: "
                f"front={command.front_output:.3f}, back={command.back_output:.3f}, "
                f"heading={gyro_reading.heading_deg:.2f}, "
                f"frame={frame.frame_id}, "
                f"lookahead=({debug['lookahead_x']:.2f}, {debug['lookahead_y']:.2f}), "
                f"speed={debug['speed_scale']:.2f}, "
                f"remaining={debug['remaining_distance']:.2f}, "
                f"edge_override={edge_correction.edge_override_active}"
            )
            if edge_correction.edge_override_active:
                print(f"  safety: {edge_correction.detail}")
            if vision_target.visible:
                print(
                    f"  vision: target={vision_target.label} "
                    f"x_offset={vision_target.x_offset_norm:.2f} "
                    f"y_offset={vision_target.y_offset_norm:.2f} "
                    f"confidence={vision_target.confidence:.2f}"
                )
            connection.send_motor_command(command)

        print(
            "\nNext hardware step: replace the simulated IR/camera events with real "
            "Arduino sensor packets and real Logitech camera detections."
        )
    finally:
        connection.stop()
        print(f"Connection status: {connection.status()}")
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
        default="/dev/ttyACM0",
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
