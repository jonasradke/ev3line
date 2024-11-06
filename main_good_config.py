#!/usr/bin/env pybricks-micropython
#Config with sensor 2cm above ground. Using trick to read both sides of line

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Color
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the EV3 brick
ev3 = EV3Brick()

# Initialize motors
left_motor = Motor(Port.B)   # Left wheel motor
right_motor = Motor(Port.C)  # Right wheel motor

# Initialize the color sensor in RGB mode
color_sensor = ColorSensor(Port.S2)

# Initialize the drive base
robot = DriveBase(left_motor, right_motor, 56, 114)

# PID tuning parameters
gain = 8               # Proportional gain
derivative_gain =6    # Derivative gain
integral_gain = 0.01    # Integral gain

# Moving average window size for sensor smoothing
smoothing_window = 5
rgb_history = [[0, 0, 0]] * smoothing_window  # History of RGB values

# Variables to store error values for PID control
previous_error = 0
integral_error = 0

def get_smoothed_rgb():
    """Get smoothed RGB values using a moving average filter."""
    # Read current RGB values
    current_rgb = color_sensor.rgb()
    
    # Update history with the latest RGB reading
    rgb_history.pop(0)  # Remove the oldest reading
    rgb_history.append(current_rgb)  # Add the latest reading

    # Calculate the average for each color channel
    smoothed_rgb = [
        sum(rgb[i] for rgb in rgb_history) // smoothing_window
        for i in range(3)
    ]
    return smoothed_rgb

while True:
    # Get the smoothed RGB values from the color sensor
    smoothed_rgb = get_smoothed_rgb()

    # Calculate the proportional error based on smoothed RGB values
    if smoothed_rgb[1] > smoothed_rgb[2]:
        proportional_error = (smoothed_rgb[1] * 2 - smoothed_rgb[2])
    else:
        proportional_error = (smoothed_rgb[2] - smoothed_rgb[1]) * -1

    # Calculate the derivative (rate of change of the error)
    derivative_error = proportional_error - previous_error

    # Calculate the integral error (accumulation of past errors)
    integral_error += proportional_error

    # Apply dynamic gain adjustment for sharper turns
    adjusted_gain = gain + (abs(proportional_error) * 0.1)

    # Calculate the turn rate with all PID components
    turn_rate = (
        (proportional_error * adjusted_gain) +
        (derivative_error * derivative_gain) +
        (integral_error * integral_gain)
    )

    # Drive forward with the calculated turn rate and adjusted speed
    robot.drive(100, turn_rate)

    # Update previous error for the next loop iteration
    previous_error = proportional_error
