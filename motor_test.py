import argparse
import time

from connection import SerialArduinoConnection
from drivetrain import MotorCommand
from robot_runner import scale_motor_command


def build_base_command(direction: str, power: float) -> MotorCommand:
    if direction == "forward":
        return MotorCommand(power, power)
    if direction == "backward":
        return MotorCommand(-power, -power)
    if direction == "left":
        return MotorCommand(-power, power)
    if direction == "right":
        return MotorCommand(power, -power)
    if direction == "front-only":
        return MotorCommand(power, 0.0)
    if direction == "back-only":
        return MotorCommand(0.0, power)
    raise ValueError("Unknown direction: {}".format(direction))


def main() -> None:
    parser = argparse.ArgumentParser(description="Send a timed motor test command to the Arduino bridge.")
    parser.add_argument(
        "direction",
        choices=["forward", "backward", "left", "right", "front-only", "back-only"],
        help="High-level motion command to test.",
    )
    parser.add_argument("--seconds", type=float, default=5.0, help="How long to hold the command.")
    parser.add_argument("--power", type=float, default=0.8, help="Base command magnitude in [0.0, 1.0].")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Arduino serial port.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument("--interval", type=float, default=0.05, help="Resend interval to satisfy the Uno watchdog.")
    args = parser.parse_args()

    if args.power < 0.0 or args.power > 1.0:
        raise ValueError("--power must be between 0.0 and 1.0")
    if args.seconds <= 0.0:
        raise ValueError("--seconds must be positive")
    if args.interval <= 0.0:
        raise ValueError("--interval must be positive")

    connection = SerialArduinoConnection(args.port, args.baud)
    connection.connect()
    try:
        base_command = build_base_command(args.direction, args.power)
        command = scale_motor_command(base_command, power_scale=1.0)
        print(
            "Testing {} for {:.2f}s -> front={:.3f}, back={:.3f}".format(
                args.direction,
                args.seconds,
                command.front_output,
                command.back_output,
            )
        )
        deadline = time.time() + args.seconds
        while time.time() < deadline:
            connection.send_motor_command(command)
            time.sleep(args.interval)
        connection.send_motor_command(MotorCommand(0.0, 0.0))
    finally:
        connection.close()


if __name__ == "__main__":
    main()
