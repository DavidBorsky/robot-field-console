"""Shared robot-side constants.

These are starter values so the software can be structured before the
physical robot is finished. Expect to tune most of them once hardware exists.
"""

FIELD_UNITS = "inches"
FIELD_WIDTH_IN = 96.0
FIELD_HEIGHT_IN = 60.0

# Current chassis assumptions based on the planned build.
WHEEL_DIAMETER_IN = 2.0
WHEEL_WIDTH_IN = 1.0
ROBOT_LENGTH_IN = 4.5
ROBOT_WIDTH_IN = 3.5

# Distance between wheel centers along each robot axis.
FRONT_BACK_WHEEL_SPACING_IN = ROBOT_LENGTH_IN
SIDE_TO_SIDE_WHEEL_SPACING_IN = ROBOT_WIDTH_IN

# Kept for compatibility with earlier differential-drive helpers.
TRACK_WIDTH_IN = SIDE_TO_SIDE_WHEEL_SPACING_IN

# Control loop timing.
DEFAULT_CONTROL_DT_S = 0.02

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
