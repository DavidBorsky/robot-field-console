# Bring-Up Checklist

Use this checklist for the first class where the hardware is assembled and tested.

## Before powering anything

- Confirm the Raspberry Pi, Arduino Uno, motor driver, battery, and camera are all present.
- Verify the battery voltage matches the intended motor driver input range.
- Confirm you have a known-good USB cable for the Arduino and power for the Pi.
- Bring the repo on a laptop with the latest `main` branch pulled.
- Keep the robot lifted or wheels off the ground for the first motor test.

## Wiring checklist

- Connect the Pi and Arduino with the expected USB serial link.
- Verify front motor driver wiring matches the Uno pin map in `README-robot.md`.
- Verify back motor driver wiring matches the Uno pin map in `README-robot.md`.
- Verify front encoder A/B wiring matches the configured Uno pins.
- Verify back encoder A/B wiring matches the configured Uno pins.
- Connect the USB camera to the Pi.
- Leave optional IR and temperature sensors disconnected unless they are fully wired and ready.

## Software bring-up

On the Raspberry Pi:

```bash
python3 robot_server.py --host 0.0.0.0 --port 8765 --camera-index -1
```

In the browser:

- Open `index.html`.
- Set the robot API URL to `http://raspberrypi.local:8765`.
- If hostname lookup fails, switch to the Pi's local IP address.

## First checks

- Load `GET /health` and confirm the server responds.
- Load `GET /robot-state` and confirm the JSON updates correctly.
- Confirm the camera section reports either a live feed or a clear failure reason.
- Confirm the UI connects without spamming errors.
- Confirm the path JSON loaded in the UI matches the expected test path.

## Safe first motion

- Start with a very short path.
- Keep the robot on blocks or with wheels free for the first command test.
- Verify the stop/reset control is working before any longer motion.
- Use low power first and watch for reversed motor direction.
- Check that forward commands actually move both front and back drive groups the intended way.
- Check that encoder counts increase and change direction as expected.

## What to validate in class

- Camera detection works on the Pi.
- Serial connection to the Uno is stable.
- Front and back motors respond to commands.
- Encoder counts are being reported back to Python.
- Odometry moves in the correct direction.
- The robot-state endpoint reflects live pose and status changes.
- The UI camera/debug panels match what the robot is doing.

## Likely first-day fixes

- Wrong serial port
- Wrong camera index
- Reversed motor polarity
- Swapped encoder channels or inverted direction
- Incorrect counts-per-revolution constant
- Wi-Fi hostname resolution issues for `raspberrypi.local`

## If something goes wrong

- Stop the motors first.
- Check `/health` and `/robot-state` before changing code.
- Test one subsystem at a time: camera, serial, motor output, encoders, then path following.
- Write down exactly what failed so the next commit can be specific and useful.
