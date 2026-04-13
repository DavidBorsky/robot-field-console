# Robot Structure

This folder is the robot-side half of the project.

The browser UI plans paths, and `robot-paths.json` is the shared format that both the UI and Python code use.

## Recommended roles

- `robot-paths.json`
  Shared path file written by the UI and read by Python.

- `robot_runner.py`
  High-level autonomous entry point on the Raspberry Pi.
  It loads the shared path data, picks `auton` or `teleop`, and coordinates the rest of the robot stack.

- `robot_server.py`
  Lightweight Pi-side HTTP API for the browser UI.
  It exposes live robot state for field tracking and accepts remote `Run`
  requests so the laptop UI can start `auton` or `teleop`.

- `pid.py`
  Reusable PID controller code for distance, heading, and alignment loops.

- `odom.py`
  Odometry state and pose updates.
  This should estimate the robot pose using encoders plus gyro heading.

- `drivetrain.py`
  High-level drive commands.
  This is the layer that turns PID outputs into front/back motor commands for the
  current two-motor linked-wheel chassis.

- `path_follower.py`
  High-level navigation logic.
  This is where Pure Pursuit, speed profiling, waypoint progression, and a future
  Ramsete controller should live.

- `gyro.py`
  Gyro or IMU wrapper.
  This should expose heading, reset, and calibration helpers.

- `camera.py`
  Camera or vision wrapper.
  This can provide target offsets, object detections, or tag-based alignment data.
  It is currently modeled as a Logitech-camera-friendly abstraction with a
  simulated mode for development.
  The Pi HTTP server currently owns the live USB camera handle so the browser
  UI can display the feed without competing with the path runner.

- `ir_sensor.py`
  Left and right IR edge sensors.
  This is the carpet safety layer that should detect the black boundary and
  temporarily override path-following commands so the robot corrects itself and
  continues instead of falling off the edge.

- `constants.py`
  PID gains, wheel diameter, track width, speed limits, camera offsets, and other tuning values.
  It now also holds the current Pi/Uno model names, 12V battery assumption,
  1:1 gear ratio, and simulated chassis response values for pre-hardware testing.

- `connection.py`
  Pi-to-Arduino or laptop-to-Pi communications.
  A common setup is laptop -> Raspberry Pi over Wi-Fi, then Raspberry Pi -> Arduino over serial.

- `arduino/uno_bridge/uno_bridge.ino`
  Arduino Uno sketch that receives `front` and `back` motor commands over serial.
  This is the low-level bridge between Python and the future motor driver wiring.

## Hardware split

- Raspberry Pi
  Runs Python, path following, PID, odometry, camera logic, and networking.

- Arduino
  Handles low-level motor and sensor I/O, then receives movement commands from the Pi.

## Confirmed hardware

- Raspberry Pi: Raspberry Pi 3 Model B v1.2 (2015)
- Arduino: Arduino Uno
- Drive battery: 12V

## Current chassis assumptions

- 2 motors total
- front motor drives both front wheels
- back motor drives both back wheels
- 4 mecanum wheels
- motor model: Tsiny geared motor
- motor nominal voltage: 12V DC
- motor free speed: 300 RPM
- motor encoders: quadrature A/B +5V/GND leads are present
- wheel diameter: 2.0 in
- wheel width: 1.0 in
- outer front/back spacing: about 4.5 in
- outer side-to-side spacing: about 3.5 in

In software, this is currently modeled as a front/back two-channel drive:

- matching front/back output -> forward or backward motion
- opposite front/back output -> left/right strafe

True rotational behavior may need to be refined after the real chassis is built and tested.

Based on the current 2-inch wheel assumption, 300 RPM corresponds to a rough
theoretical wheel-edge speed of about 31.4 in/s before real-world load losses.

## Data flow

1. Build a path in the UI.
2. Save/export the shared JSON structure.
3. Put that payload into `robot-paths.json`.
4. Start `robot_server.py` on the Raspberry Pi.
5. The browser UI connects to the Pi server, pushes the current path, and sends a run request.
6. `robot_runner.py` loads the chosen path and uses PID, odometry, drivetrain, gyro, camera, and connection code to follow it.

In simulated mode, `robot_runner.py` now also advances a virtual pose from the
generated motor commands. That gives a much more useful pre-hardware test loop
than repeatedly commanding the same stationary pose.

During a run, the server publishes live pose updates so the field UI can show
the robot marker moving in real time.

## Live server

Default Pi robot server address:

- `http://raspberrypi.local:8765/`

Useful endpoints:

- `http://raspberrypi.local:8765/health`
- `http://raspberrypi.local:8765/robot-state`

If `raspberrypi.local` does not resolve on your network, replace it with the
Pi's local IP address, for example:

- `http://192.168.1.42:8765/`

## Encoder note

The Python and Arduino code now assume front/back encoder telemetry will be sent
back to the Pi. One constant still needs the exact motor datasheet value before
the math is truly final:

- `ENCODER_COUNTS_PER_OUTPUT_REV` in `constants.py`

It is currently set to `360.0` as a placeholder until the exact counts-per-rev
for the Tsiny motor encoder is confirmed. That value also needs to match the
actual counting mode used in the Uno sketch. Right now the sketch counts changes
on encoder channel `A` and reads channel `B` for direction, so the real number
must match that measurement method.

## Current Arduino protocol

Python currently sends motor commands like:

```text
M,<front_output>,<back_output>
```

Example:

```text
M,0.5000,-0.2000
```

The Arduino Uno sketch stores those values, clamps them to `[-1.0, 1.0]`, and
is ready to map them onto the real front/back motor driver pins once the wiring
is finalized.

The Uno sketch is also now structured to report sensor packets like:

```text
S,<left_edge>,<right_edge>,<heading_deg>,<front_count>,<back_count>,<front_rpm>,<back_rpm>,<front_temp_c>,<back_temp_c>,<battery_voltage>
```

## Current Uno pin map

For the current full L298N + two-encoder layout, the Uno sketch assumes:

- Front motor PWM `ENA` -> Arduino `9`
- Front motor direction `IN1` -> Arduino `7`
- Front motor direction `IN2` -> Arduino `6`
- Back motor PWM `ENB` -> Arduino `10`
- Back motor direction `IN3` -> Arduino `8`
- Back motor direction `IN4` -> Arduino `11`
- Front encoder `A` -> Arduino `2`
- Front encoder `B` -> Arduino `4`
- Back encoder `A` -> Arduino `3`
- Back encoder `B` -> Arduino `5`

Optional inputs are still disabled by default until they are physically wired:

- left IR edge sensor
- right IR edge sensor
- battery voltage analog sense

The Uno sketch now also includes a command watchdog, so if fresh motor commands
stop arriving for about `250 ms`, it hard-stops both motor channels.

## Edge safety behavior

The two IR sensors are planned as left/right edge detectors, not line followers.

- if the left sensor sees the black carpet edge, the robot should correct right
- if the right sensor sees the black carpet edge, the robot should correct left
- if both sensors see the edge, the robot should stop or back away briefly
- once the robot is safely back on carpet, it should continue following the path

## Shared JSON format

```json
{
  "paths": {
    "auton": [
      [12.0, 8.0],
      [30.0, 8.0],
      [30.0, 40.0]
    ],
    "teleop": [
      [10.0, 10.0],
      [20.0, 18.0]
    ]
  },
  "units": "inches",
  "field": {
    "width": 96,
    "height": 60
  }
}
```
