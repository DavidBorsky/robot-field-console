import argparse
import json
import threading
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from socketserver import ThreadingMixIn
from typing import Any, Dict, Optional, Type
from urllib.parse import urlsplit

from camera import UnavailableCamera, create_camera
from constants import DEFAULT_CAMERA_INDEX, DEFAULT_POWER_SCALE, DEFAULT_SERIAL_PORT_LINUX
from robot_runner import BASE_DIR, run_path


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def camera_status_payload(camera: Any) -> Dict[str, Any]:
    try:
        status = camera.status()
    except Exception as exc:  # pragma: no cover - defensive
        return {
            "connected": False,
            "mode": "error",
            "detail": str(exc),
            "index": None,
        }

    return {
        "connected": bool(status.connected),
        "mode": status.mode,
        "detail": status.detail,
        "index": status.index,
    }


class RobotStateStore:
    def __init__(self) -> None:
        self.snapshot = {
            "connected": False,
            "running": False,
            "mode": "auton",
            "status": "idle",
            "detail": "server ready",
            "connection": "serial",
            "pose": {"x": 0.0, "y": 0.0, "heading_deg": 0.0},
            "telemetry": {
                "front_motor_temp_c": None,
                "back_motor_temp_c": None,
                "battery_voltage": None,
            },
            "velocity_in_per_s": 0.0,
            "motor_rpm_estimate": 0.0,
            "velocity_source": "model",
            "step": 0,
            "max_steps": 0,
            "updated_at": utc_timestamp(),
            "camera": {
                "connected": False,
                "mode": "unknown",
                "detail": "camera not initialized",
                "index": None,
            },
        }
        self._lock = threading.Lock()
        self._run_thread = None  # type: Optional[threading.Thread]
        self._stop_event = None  # type: Optional[threading.Event]

    def get(self) -> Dict[str, Any]:
        with self._lock:
            return json.loads(json.dumps(self.snapshot))

    def update(self, **updates: Any) -> None:
        with self._lock:
            for key, value in updates.items():
                if key == "pose" and isinstance(value, dict):
                    self.snapshot["pose"] = {**self.snapshot.get("pose", {}), **value}
                else:
                    self.snapshot[key] = value
            self.snapshot["updated_at"] = utc_timestamp()

    def is_running(self) -> bool:
        with self._lock:
            return bool(self.snapshot.get("running"))

    def set_thread(self, thread: Optional[threading.Thread]) -> None:
        with self._lock:
            self._run_thread = thread

    def set_stop_event(self, stop_event: Optional[threading.Event]) -> None:
        with self._lock:
            self._stop_event = stop_event

    def stop_run(self, timeout_s: float = 2.0) -> bool:
        with self._lock:
            thread = self._run_thread
            stop_event = self._stop_event
        if stop_event is not None:
            stop_event.set()
        if thread is not None and thread.is_alive():
            thread.join(timeout_s)
        with self._lock:
            return not (self._run_thread is not None and self._run_thread.is_alive())


def build_handler(
    state_store: RobotStateStore,
    camera: Any,
    default_path_file: Path,
    default_serial_port: str,
    default_baud: int,
) -> Type[BaseHTTPRequestHandler]:
    class RobotRequestHandler(BaseHTTPRequestHandler):
        server_version = "WaypointRobotServer/0.1"

        def _send_json(self, payload: Dict[str, Any], status: int = HTTPStatus.OK) -> None:
            raw = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()
            self.wfile.write(raw)

        def _read_json_body(self) -> Dict[str, Any]:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0:
                return {}
            payload = self.rfile.read(content_length)
            if not payload:
                return {}
            return json.loads(payload.decode("utf-8"))

        def do_OPTIONS(self) -> None:  # noqa: N802
            self._send_json({"ok": True})

        def do_GET(self) -> None:  # noqa: N802
            request_path = urlsplit(self.path).path
            if request_path == "/health":
                self._send_json({"ok": True, "status": "ready", "camera": camera_status_payload(camera)})
                return
            if request_path == "/robot-state":
                snapshot = state_store.get()
                snapshot["connected"] = True
                snapshot["camera"] = camera_status_payload(camera)
                self._send_json(snapshot)
                return
            if request_path == "/camera":
                try:
                    frame = camera.read_frame()
                    if getattr(frame, "jpeg_bytes", None) is not None:
                        img_bytes = frame.jpeg_bytes
                        self.send_response(HTTPStatus.OK)
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(img_bytes)))
                        self.send_header("Access-Control-Allow-Origin", "*")
                        self.end_headers()
                        self.wfile.write(img_bytes)
                    elif frame.frame_data is not None:
                        import cv2
                        # Encode frame as JPEG
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                        _, encoded_img = cv2.imencode('.jpg', frame.frame_data, encode_param)
                        img_bytes = encoded_img.tobytes()
                        
                        self.send_response(HTTPStatus.OK)
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(img_bytes)))
                        self.send_header("Access-Control-Allow-Origin", "*")
                        self.end_headers()
                        self.wfile.write(img_bytes)
                    else:
                        # For simulated camera, return a placeholder
                        img_bytes = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c\x1c $.\' \",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x11\x08\x01\xe0\x02\x80\x03\x01\"\x00\x02\x11\x01\x03\x11\x01\xff\xc4\x00\x14\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x08\xff\xc4\x00\x14\x10\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff\xda\x00\x0c\x03\x01\x00\x02\x11\x03\x11\x00\x3f\x00\xaa\xaa\xaa\xaa\xff\xd9'
                        self.send_response(HTTPStatus.OK)
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(img_bytes)))
                        self.send_header("Access-Control-Allow-Origin", "*")
                        self.end_headers()
                        self.wfile.write(img_bytes)
                except Exception as e:
                    self._send_json({"error": str(e)}, status=HTTPStatus.INTERNAL_SERVER_ERROR)
                return
            self._send_json({"error": "not found"}, status=HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            if self.path == "/reset":
                try:
                    payload = self._read_json_body()
                except json.JSONDecodeError:
                    self._send_json({"error": "invalid JSON body"}, status=HTTPStatus.BAD_REQUEST)
                    return

                if not state_store.stop_run(timeout_s=2.0):
                    self._send_json({"error": "robot did not stop in time"}, status=HTTPStatus.CONFLICT)
                    return

                reset_pose = payload.get("pose") or {}
                pose = {
                    "x": float(reset_pose.get("x", 0.0)),
                    "y": float(reset_pose.get("y", 0.0)),
                    "heading_deg": float(reset_pose.get("heading_deg", 0.0)),
                }
                mode = payload.get("mode", state_store.get().get("mode", "auton"))
                state_store.update(
                    running=False,
                    mode=mode,
                    status="idle",
                    detail="pose reset from UI",
                    pose=pose,
                    velocity_in_per_s=0.0,
                    motor_rpm_estimate=0.0,
                    velocity_source="encoder",
                    step=0,
                    max_steps=0,
                )
                self._send_json({"ok": True, "status": "reset", "pose": pose})
                return

            if self.path != "/run":
                self._send_json({"error": "not found"}, status=HTTPStatus.NOT_FOUND)
                return

            if state_store.is_running():
                self._send_json(
                    {"error": "robot is already running"},
                    status=HTTPStatus.CONFLICT,
                )
                return

            try:
                payload = self._read_json_body()
            except json.JSONDecodeError:
                self._send_json({"error": "invalid JSON body"}, status=HTTPStatus.BAD_REQUEST)
                return

            mode = payload.get("mode", "auton")
            connection = payload.get("connection", "serial")
            follower = payload.get("follower", "pure_pursuit")
            path_file = Path(payload.get("path_file") or default_path_file)
            serial_port = payload.get("port", default_serial_port)
            serial_baud = int(payload.get("baud", default_baud))
            power_scale = float(payload.get("power_scale", DEFAULT_POWER_SCALE))
            robot_paths = payload.get("robot_paths")

            if robot_paths is not None:
                path_file.parent.mkdir(parents=True, exist_ok=True)
                path_file.write_text(json.dumps(robot_paths, indent=2) + "\n", encoding="utf-8")

            state_store.update(
                running=True,
                mode=mode,
                connection=connection,
                status="starting",
                detail="launching {} run".format(mode),
                path_file=str(path_file),
                power_scale=power_scale,
            )

            def publish(update: Dict[str, Any]) -> None:
                state_store.update(**update)

            stop_event = threading.Event()

            def worker() -> None:
                try:
                    run_path(
                        mode=mode,
                        path_file=path_file,
                        simulate_connection=(connection == "simulated"),
                        follower_controller=follower,
                        serial_port=serial_port,
                        serial_baud=serial_baud,
                        power_scale=power_scale,
                        status_callback=publish,
                        stop_event=stop_event,
                    )
                    final_state = state_store.get()
                    if final_state.get("status") not in {"complete", "stopped"}:
                        state_store.update(status="complete", detail="run finished")
                except Exception as exc:  # pragma: no cover - surfaced over API
                    state_store.update(
                        status="error",
                        detail=str(exc),
                    )
                finally:
                    state_store.update(running=False)
                    state_store.set_thread(None)
                    state_store.set_stop_event(None)

            thread = threading.Thread(target=worker, name="robot-runner", daemon=True)
            state_store.set_thread(thread)
            state_store.set_stop_event(stop_event)
            thread.start()

            self._send_json(
                {
                    "ok": True,
                    "status": "starting",
                    "mode": mode,
                    "connection": connection,
                },
                status=HTTPStatus.ACCEPTED,
            )

        def log_message(self, format: str, *args: Any) -> None:
            return

    return RobotRequestHandler


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve live robot state to the field UI.")
    parser.add_argument("--host", default="0.0.0.0", help="Host interface to bind")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port to bind")
    parser.add_argument(
        "--path-file",
        default=str(BASE_DIR / "robot-paths.json"),
        help="Path to robot-paths.json on the Pi",
    )
    parser.add_argument(
        "--serial-port",
        default=DEFAULT_SERIAL_PORT_LINUX,
        help="Default serial port for the Arduino bridge",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Default serial baud rate")
    parser.add_argument("--simulate", action="store_true", help="Use simulated camera instead of hardware")
    parser.add_argument(
        "--camera-index",
        type=int,
        default=DEFAULT_CAMERA_INDEX,
        help="Camera index to open. Use -1 to auto-detect across common USB camera indices.",
    )
    args = parser.parse_args()

    state_store = RobotStateStore()
    camera = create_camera(simulate=args.simulate, index=args.camera_index)
    try:
        camera.start()
    except Exception as exc:
        camera = UnavailableCamera(str(exc))

    state_store.update(camera=camera_status_payload(camera))
    
    handler = build_handler(
        state_store=state_store,
        camera=camera,
        default_path_file=Path(args.path_file),
        default_serial_port=args.serial_port,
        default_baud=args.baud,
    )
    server = ThreadedHTTPServer((args.host, args.port), handler)
    print("Robot server listening on http://{}:{}".format(args.host, args.port))
    server.serve_forever()


if __name__ == "__main__":
    main()
