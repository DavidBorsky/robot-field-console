"""Connection layer between the Raspberry Pi and the Arduino.

This starts with two modes:
- simulated output for development without hardware
- optional serial output for the real Pi/Arduino link
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

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
class SensorSnapshot:
    ir: IRSensorState = IRSensorState()
    heading_deg: float | None = None


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
        return self.sensor_snapshot

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

    def connect(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.serial_handle = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def _require_handle(self):
        if self.serial_handle is None:
            raise RuntimeError("SerialArduinoConnection is not connected")
        return self.serial_handle

    def send_motor_command(self, command: MotorCommand) -> None:
        handle = self._require_handle()
        payload = f"M,{command.front_output:.4f},{command.back_output:.4f}\n"
        handle.write(payload.encode("utf-8"))

    def read_sensors(self) -> SensorSnapshot:
        # Placeholder until the Arduino sends back real sensor packets.
        return SensorSnapshot()

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


def create_connection(simulate: bool = True) -> RobotConnection:
    if simulate:
        return SimulatedConnection()
    return SerialArduinoConnection()
