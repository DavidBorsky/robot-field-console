# robot-field-console

Browser-based robot path planner with a live field UI, Raspberry Pi robot
server, and shared waypoint format for autonomous path testing.

Live UI: `https://davidborsky.github.io/robot-field-console/`

The default layout is a 96x60 inch field, but the planner is not locked to that
size. You can adapt the field dimensions, waypoint file, and robot endpoint for
your own robot project.

## What this repo includes

- `index.html`
  Browser-based field planner and live robot dashboard.

- `robot_server.py`
  Raspberry Pi HTTP server that exposes robot state, camera status, and run
  controls to the UI.

- `robot_runner.py`
  Autonomous runner that loads waypoint paths and drives the robot stack.

- `robot-paths.json`
  Shared waypoint file used by both the UI and robot code.

- `README-robot.md`
  Robot-side architecture, hardware notes, and protocol details.

- `BRINGUP.md`
  First-hardware-day checklist for wiring, validation, and safe testing.

## Customizing the field

This project is meant to be reusable.

- The default field is `96 x 60` inches.
- You can change the field dimensions for a different course or robot space.
- You can reuse the same UI with your own Raspberry Pi server by changing the robot API URL.
- You can update `robot-paths.json` with your own saved paths and units.

## Quick start

### 1. Open the UI

Open `index.html` in your browser.

Use the planner to create or edit paths, then save them into `robot-paths.json`.

### 2. Run the robot server locally

From this folder:

```bash
python3 robot_server.py --simulate --host 0.0.0.0 --port 8765
```

Then connect the UI to:

```text
http://localhost:8765
```

Useful endpoints:

- `/health`
- `/robot-state`
- `/camera`

### 3. Run on the Raspberry Pi

For hardware bring-up, start the server on the Pi with camera auto-detection:

```bash
python3 robot_server.py --host 0.0.0.0 --port 8765 --camera-index -1
```

Then connect the browser UI to:

```text
http://raspberrypi.local:8765
```

If `raspberrypi.local` does not resolve, use the Pi's local IP address instead.

## Connect to your own Raspberry Pi

Anyone reusing this project can point the UI at their own Pi-hosted robot server.

1. Start `robot_server.py` on the Raspberry Pi.
2. Open `index.html` in a browser on the laptop.
3. Enter the Pi server URL in the robot API field.
4. Click `Connect`.

Examples:

```text
http://raspberrypi.local:8765
http://192.168.1.42:8765
```

The UI uses that base URL for:

- `/health`
- `/robot-state`
- `/camera`

To share this project with someone else, they only need:

- this repo
- a running Raspberry Pi server from `robot_server.py`
- the correct Pi hostname or local IP address

## Web UI link

This repo can also host the browser UI with GitHub Pages.

Expected project URL:

```text
https://davidborsky.github.io/robot-field-console/
```

Once GitHub Pages is enabled for this repository, that link should load the same
`index.html` planner UI directly from the web.

If you want to use the hosted UI with your own Raspberry Pi:

1. Open the GitHub Pages URL in your browser.
2. Enter your Pi server URL in the robot API field.
3. Click `Connect`.

Example Pi URLs:

```text
http://raspberrypi.local:8765
http://192.168.1.42:8765
```

## Recommended workflow

1. Build or adjust a path in the UI.
2. Save the path to `robot-paths.json`.
3. Start `robot_server.py`.
4. Confirm `/health` and `/robot-state` respond.
5. Connect the UI to the server.
6. Verify the robot marker, status panel, and camera panel update correctly.
7. Run `auton` or `teleop`.

- `--simulate` is the safest way to test the full browser/server flow on just a laptop.
- `--camera-index -1` tries common USB camera indices automatically.
- If no camera is found, the server should stay up and report camera status instead of crashing.
