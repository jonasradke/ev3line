#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the EV3 brick.
ev3 = EV3Brick()

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S2)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Calculate the light threshold.
BLACK = 6
WHITE = 46
threshold = (BLACK + WHITE) / 2

# Set the drive speed.
DRIVE_SPEED = 80

# PID constants
PROPORTIONAL_GAIN = 1.9
INTEGRAL_GAIN = 0.06
DERIVATIVE_GAIN = 0.2

# Initialize variables for the PID controller
integral = 0
last_error = 0
blue_detected = False
blue_detected2 = True

#while not ev3.buttons.pressed():
        #wait(10)


# Start following the line endlessly.
while True:
    # Get the current RGB value from the sensor
    current_rgb = line_sensor.rgb()

    # Compare the current RGB value with the calibrated blue RGB values

    if (abs(current_rgb[0] < 10) and 
        abs(current_rgb[1] < 15) and 
        abs(current_rgb[2] > 30)) and not blue_detected:
        # If the color is close enough to blue, perform actions like moving forward and turning
        robot.straight(20)
        robot.turn(50)
        integral = 0
        blue_detected = True  # Set flag to prevent multiple triggers

    if (abs(current_rgb[0] < 10) and 
        abs(current_rgb[1] < 15) and 
        abs(current_rgb[2] > 30)) and not blue_detected2:
        # If the color is close enough to blue, perform actions like moving forward and turning
        robot.straight(50)
        blue_detected2 = True
        

    if (abs(current_rgb[0] < 14) and 
        abs(current_rgb[1] > 20) and 
        abs(current_rgb[2] < 20)):
        robot.stop()
        wait(2000)
        blue_detected2 = False


    # Get the current light reflection and calculate the deviation from the threshold.
    light_value = line_sensor.reflection()
    error = light_value - threshold

    # Update integral and derivative terms
    integral += error
    derivative = error - last_error

    # Calculate turn rate using the PID formula
    turn_rate = (PROPORTIONAL_GAIN * error) + (INTEGRAL_GAIN * integral) + (DERIVATIVE_GAIN * derivative)

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # Update the last error for the next loop iteration
    last_error = error
