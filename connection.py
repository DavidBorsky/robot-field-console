"""Connection layer between the Raspberry Pi and the Arduino.

This starts with two modes:
- simulated output for development without hardware
- optional serial output for the real Pi/Arduino link
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Optional

try:
    from typing import Protocol
except ImportError:  # pragma: no cover - Python < 3.8 fallback
    class Protocol(object):
        pass

from constants import (
    ENCODER_COUNTS_PER_OUTPUT_REV,
    ENCODER_SAMPLE_WINDOW_S,
    MOTOR_FREE_SPEED_RPM,
)
from drivetrain import MotorCommand
from ir_sensor import IRSensorState

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    serial = None


@dataclass(frozen=True)
class RobotStatus:
    connected: bool
    mode: str
    detail: str


@dataclass(frozen=True)
class EncoderSnapshot:
    front_count: int = 0
    back_count: int = 0
    front_rpm: float = 0.0
    back_rpm: float = 0.0
    counts_per_rev: float = ENCODER_COUNTS_PER_OUTPUT_REV


@dataclass(frozen=True)
class RobotTelemetry:
    front_motor_temp_c: Optional[float] = None
    back_motor_temp_c: Optional[float] = None
    battery_voltage: Optional[float] = None


@dataclass(frozen=True)
class SensorSnapshot:
    ir: IRSensorState = IRSensorState()
    heading_deg: Optional[float] = None
    encoders: EncoderSnapshot = EncoderSnapshot()
    telemetry: RobotTelemetry = RobotTelemetry()


class RobotConnection(Protocol):
    def connect(self) -> None: ...
    def send_motor_command(self, command: MotorCommand) -> None: ...
    def read_sensors(self) -> SensorSnapshot: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...
    def status(self) -> RobotStatus: ...


class SimulatedConnection:
    """Development connection that logs commands instead of sending them."""

    def __init__(self):
        self.connected = False
        self.last_command = MotorCommand(front_output=0.0, back_output=0.0)
        self.sensor_snapshot = SensorSnapshot()
        self._last_sensor_time = time.monotonic()
        self._front_count_float = 0.0
        self._back_count_float = 0.0

    def connect(self) -> None:
        self.connected = True
        print("[sim] connected")

    def send_motor_command(self, command: MotorCommand) -> None:
        if not self.connected:
            raise RuntimeError("SimulatedConnection is not connected")
        self.last_command = command
        print(
            f"[sim] motor command -> front={command.front_output:.3f}, "
            f"back={command.back_output:.3f}"
        )

    def set_sensor_snapshot(self, snapshot: SensorSnapshot) -> None:
        self.sensor_snapshot = snapshot

    def read_sensors(self) -> SensorSnapshot:
        if not self.connected:
            raise RuntimeError("SimulatedConnection is not connected")
        now = time.monotonic()
        dt = max(now - self._last_sensor_time, 1e-6)
        self._last_sensor_time = now

        front_rpm = self.last_command.front_output * MOTOR_FREE_SPEED_RPM
        back_rpm = self.last_command.back_output * MOTOR_FREE_SPEED_RPM
        counts_per_second = ENCODER_COUNTS_PER_OUTPUT_REV / 60.0
        self._front_count_float += front_rpm * counts_per_second * dt
        self._back_count_float += back_rpm * counts_per_second * dt

        return SensorSnapshot(
            ir=self.sensor_snapshot.ir,
            heading_deg=self.sensor_snapshot.heading_deg,
            encoders=EncoderSnapshot(
                front_count=int(round(self._front_count_float)),
                back_count=int(round(self._back_count_float)),
                front_rpm=front_rpm,
                back_rpm=back_rpm,
                counts_per_rev=ENCODER_COUNTS_PER_OUTPUT_REV,
            ),
            telemetry=self.sensor_snapshot.telemetry,
        )

    def stop(self) -> None:
        self.send_motor_command(MotorCommand(front_output=0.0, back_output=0.0))

    def close(self) -> None:
        if self.connected:
            print("[sim] disconnected")
        self.connected = False

    def status(self) -> RobotStatus:
        detail = (
            f"last front={self.last_command.front_output:.3f}, "
            f"back={self.last_command.back_output:.3f}"
        )
        return RobotStatus(connected=self.connected, mode="simulated", detail=detail)


class SerialArduinoConnection:
    """Pi-to-Arduino serial connection.

    The exact Arduino protocol can still evolve, but this gives us a stable
    place to send front/back motor commands once hardware is present.
    """

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200, timeout: float = 0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_handle = None
        self.latest_sensor_snapshot = SensorSnapshot()

    def connect(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.serial_handle = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self.latest_sensor_snapshot = SensorSnapshot()
        # Arduino Uno often resets when the serial port is opened.
        time.sleep(2.0)
        self.serial_handle.reset_input_buffer()
        self.serial_handle.reset_output_buffer()

    def _require_handle(self):
        if self.serial_handle is None:
            raise RuntimeError("SerialArduinoConnection is not connected")
        return self.serial_handle

    def _parse_sensor_packet(self, line: str) -> Optional[SensorSnapshot]:
        if not line.startswith("S,"):
            return None

        parts = [part.strip() for part in line.split(",")]
        if len(parts) < 11:
            return None

        try:
            left_edge = bool(int(parts[1]))
            right_edge = bool(int(parts[2]))
            heading_deg = None if parts[3] == "" else float(parts[3])
            front_count = int(parts[4])
            back_count = int(parts[5])
            front_rpm = float(parts[6])
            back_rpm = float(parts[7])
            front_temp_c = None if parts[8] == "" else float(parts[8])
            back_temp_c = None if parts[9] == "" else float(parts[9])
            battery_voltage = None if parts[10] == "" else float(parts[10])
        except ValueError:
            return None

        return SensorSnapshot(
            ir=IRSensorState(
                left_edge_detected=left_edge,
                right_edge_detected=right_edge,
            ),
            heading_deg=heading_deg,
            encoders=EncoderSnapshot(
                front_count=front_count,
                back_count=back_count,
                front_rpm=front_rpm,
                back_rpm=back_rpm,
                counts_per_rev=ENCODER_COUNTS_PER_OUTPUT_REV,
            ),
            telemetry=RobotTelemetry(
                front_motor_temp_c=front_temp_c,
                back_motor_temp_c=back_temp_c,
                battery_voltage=battery_voltage,
            ),
        )

    def _drain_input(self, duration_s: Optional[float] = None) -> None:
        handle = self._require_handle()
        deadline = None if duration_s is None else time.monotonic() + duration_s
        while True:
            if deadline is not None and time.monotonic() >= deadline and handle.in_waiting <= 0:
                break
            if handle.in_waiting <= 0:
                if deadline is None:
                    break
                line = handle.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
            else:
                line = handle.readline().decode("utf-8", errors="replace").strip()
            if not line:
                if deadline is None:
                    break
                continue
            parsed = self._parse_sensor_packet(line)
            if parsed is not None:
                self.latest_sensor_snapshot = parsed
            else:
                print(f"[serial] {line}")
            if deadline is None and handle.in_waiting <= 0:
                break

    def send_motor_command(self, command: MotorCommand) -> None:
        handle = self._require_handle()
        payload = f"M,{command.front_output:.4f},{command.back_output:.4f}\n"
        handle.write(payload.encode("utf-8"))
        handle.flush()
        self._drain_input(duration_s=0.02)

    def read_sensors(self) -> SensorSnapshot:
        self._drain_input(duration_s=ENCODER_SAMPLE_WINDOW_S)
        return self.latest_sensor_snapshot

    def stop(self) -> None:
        self.send_motor_command(MotorCommand(front_output=0.0, back_output=0.0))

    def close(self) -> None:
        if self.serial_handle is not None:
            self.serial_handle.close()
            self.serial_handle = None

    def status(self) -> RobotStatus:
        connected = self.serial_handle is not None
        detail = f"port={self.port} baud={self.baudrate}"
        return RobotStatus(connected=connected, mode="serial", detail=detail)


def create_connection(
    simulate: bool = True,
    port: str = "/dev/ttyACM0",
    baudrate: int = 115200,
) -> RobotConnection:
    if simulate:
        return SimulatedConnection()
    return SerialArduinoConnection(port=port, baudrate=baudrate)


def run_serial_smoke_test(port: str, baudrate: int = 115200) -> None:
    connection = SerialArduinoConnection(port=port, baudrate=baudrate)
    connection.connect()
    try:
        print(f"Connected to Arduino on {port} at {baudrate} baud")
        connection.send_motor_command(MotorCommand(front_output=0.5, back_output=-0.2))
        connection.send_motor_command(MotorCommand(front_output=0.0, back_output=0.0))
        print(f"Status: {connection.status()}")
    finally:
        connection.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the Arduino serial connection.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM3 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    args = parser.parse_args()
    run_serial_smoke_test(port=args.port, baudrate=args.baud)
