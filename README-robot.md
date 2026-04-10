# Robot Structure

This folder is the robot-side half of the project.

The browser UI plans paths, and `robot-paths.json` is the shared format that both the UI and Python code use.

## Recommended roles

- `robot-paths.json`
  Shared path file written by the UI and read by Python.

- `robot-runner.py`
  High-level autonomous entry point on the Raspberry Pi.
  It loads the shared path data, picks `auton` or `teleop`, and coordinates the rest of the robot stack.

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

- `ir_sensor.py`
  Left and right IR edge sensors.
  This is the carpet safety layer that should detect the black boundary and
  temporarily override path-following commands so the robot corrects itself and
  continues instead of falling off the edge.

- `constants.py`
  PID gains, wheel diameter, track width, speed limits, camera offsets, and other tuning values.

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

## Current chassis assumptions

- 2 motors total
- front motor drives both front wheels
- back motor drives both back wheels
- 4 mecanum wheels
- wheel diameter: 2.0 in
- wheel width: 1.0 in
- outer front/back spacing: about 4.5 in
- outer side-to-side spacing: about 3.5 in

In software, this is currently modeled as a front/back two-channel drive:

- matching front/back output -> forward or backward motion
- opposite front/back output -> left/right strafe

True rotational behavior may need to be refined after the real chassis is built and tested.

## Data flow

1. Build a path in the UI.
2. Save/export the shared JSON structure.
3. Put that payload into `robot-paths.json`.
4. Run `robot-runner.py` on the Raspberry Pi.
5. `robot-runner.py` loads the chosen path and uses PID, odometry, drivetrain, gyro, camera, and connection code to follow it.

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
