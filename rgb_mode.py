#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import ColorSensor, Motor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the motors and color sensor
left_motor = Motor(Port.B)  # Left motor connected to port B
right_motor = Motor(Port.C)  # Right motor connected to port C
color_sensor = ColorSensor(Port.S2)  # Color sensor connected to port S3

# Initialize the drive base
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# Set the calibration values for the line and background (adjust as needed)
LINE_RGB = (2, 2, 4)      # RGB values for the line (e.g., black)
BACKGROUND_RGB = (18, 21, 32)  # RGB values for the background (e.g., white)
THRESHOLD = 8  # Threshold to differentiate line from background

# Base parameters for line following
BASE_SPEED = 50  # Speed of the robot
TURN_GAIN = 1   # Gain for proportional control
D_GAIN = 0.2      # Derivative gain

# Initialize previous error for derivative calculation
previous_error = 0

def calculate_error(current_rgb, target_rgb):
    """
    Calculate the Euclidean distance as the error between current RGB and target RGB.
    """
    error = sum((current - target) ** 2 for current, target in zip(current_rgb, target_rgb)) ** 0.5
    return error

def calculate_turn_direction(current_rgb):

    direction = current_rgb[0]-current_rgb[2]
    if current_rgb[0]-current_rgb[2]>0:
        return 1
    else: 
        return -1

# Main loop for line following with PD control
while True:
    # Read the current RGB values from the color sensor
    current_rgb = color_sensor.rgb()

    # Calculate the proportional error
    error = calculate_error(current_rgb, LINE_RGB)

    turn_direction = calculate_turn_direction(current_rgb)

    # Calculate the derivative error
    derivative = error - previous_error

    # Calculate the turn rate based on PD control
    turn = (TURN_GAIN * error) + (D_GAIN * derivative) * turn_direction

    # Update the previous error for the next iteration
    previous_error = error

    # Check if the error is above the threshold for turning
    if error > THRESHOLD:
        # Adjust turning based on PD control
        robot.drive(BASE_SPEED, turn)
    else:
        # Drive straight when on the line
        robot.drive(BASE_SPEED, 0)

    # Short delay for stable readings
    wait(10)