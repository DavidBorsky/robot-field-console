"""Shared robot-side constants.

These are starter values so the software can be structured before the
physical robot is finished. Expect to tune most of them once hardware exists.
"""

from math import pi

FIELD_UNITS = "inches"
FIELD_WIDTH_IN = 96.0
FIELD_HEIGHT_IN = 60.0
PLAYABLE_MIN_X_IN = 1.9
PLAYABLE_MAX_X_IN = 94.0
PLAYABLE_MIN_Y_IN = 1.8
PLAYABLE_MAX_Y_IN = 58.0

# Platform details known so far.
DRIVE_BATTERY_VOLTAGE = 12.0
GEAR_RATIO = 1.0
RASPBERRY_PI_MODEL = "Raspberry Pi 3 Model B v1.2"
ARDUINO_MODEL = "Arduino Uno"
MOTOR_MODEL = "Tsiny geared motor"
MOTOR_NOMINAL_VOLTAGE = 12.0
MOTOR_FREE_SPEED_RPM = 300.0
DEFAULT_SERIAL_PORT_LINUX = "/dev/ttyACM0"
DEFAULT_SERIAL_PORT_WINDOWS = "COM3"
DEFAULT_CAMERA_INDEX = -1

# Current chassis assumptions based on the planned build.
WHEEL_DIAMETER_IN = 2.0
WHEEL_WIDTH_IN = 1.0
ROBOT_LENGTH_IN = 4.5
ROBOT_WIDTH_IN = 3.5
WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * pi
MOTOR_FREE_SPEED_RPS = MOTOR_FREE_SPEED_RPM / 60.0
THEORETICAL_MAX_WHEEL_SPEED_IN_PER_S = WHEEL_CIRCUMFERENCE_IN * MOTOR_FREE_SPEED_RPS / GEAR_RATIO

# Distance between wheel centers along each robot axis.
FRONT_BACK_WHEEL_SPACING_IN = ROBOT_LENGTH_IN
SIDE_TO_SIDE_WHEEL_SPACING_IN = ROBOT_WIDTH_IN

# Front/back linked mecanum odometry calibration. These will likely need
# hardware tuning once real encoder and strafe data are available.
ODOM_FORWARD_SCALE = 1.0
ODOM_STRAFE_SCALE = 1.0
ENCODER_COUNTS_PER_OUTPUT_REV = 360.0
ENCODER_SAMPLE_WINDOW_S = 0.05

# Kept for compatibility with earlier differential-drive helpers.
TRACK_WIDTH_IN = SIDE_TO_SIDE_WHEEL_SPACING_IN

# Control loop timing.
DEFAULT_CONTROL_DT_S = 0.02
DEFAULT_POWER_SCALE = 0.5

# Simulated chassis response for pre-hardware tuning.
SIM_MAX_FORWARD_SPEED_IN_PER_S = 8.0
SIM_MAX_STRAFE_SPEED_IN_PER_S = 6.0
SIM_MAX_STEPS_PER_RUN = 120

# Path-speed profiling defaults.
FOLLOWER_LONG_RUN_DISTANCE_IN = 24.0
FOLLOWER_SLOWDOWN_DISTANCE_IN = 18.0
FOLLOWER_ACCEL_RAMP_PER_S = 1.8
FOLLOWER_DECEL_RAMP_PER_S = 3.4
FOLLOWER_CORNER_SLOWDOWN_GAIN = 1.2
FOLLOWER_CORNER_STOP_DURATION_S = 0.5
FOLLOWER_CORNER_STOP_DISTANCE_IN = 0.6
FOLLOWER_STOP_BRAKE_DISTANCE_IN = 16.0

# Translation PID starter gains.
DRIVE_KP = 0.8
DRIVE_KI = 0.0
DRIVE_KD = 0.08

# Heading PID starter gains.
TURN_KP = 2.2
TURN_KI = 0.0
TURN_KD = 0.12

# Safety limits for controller outputs and integral accumulation.
MAX_DRIVE_OUTPUT = 1.0
MAX_TURN_OUTPUT = 1.0
MAX_INTEGRAL = 10.0
