#!/usr/bin/env pybricks-micropython
# Import the necessary modules from the Pybricks library
from pybricks.hubs import EV3Brick
from pybricks.motor import Motor, MoveSteering
from pybricks.sensor import ColorSensor
from pybricks.tools import wait

# Initialize the EV3 Brick, motors, and color sensor
ev3 = EV3Brick()
steering_drive = MoveSteering(Port.B, Port.C)  # Set motor ports
color_sensor = ColorSensor(Port.S2)  # Set color sensor port

# Set the threshold for line detection
THRESHOLD = 50  # Adjust this threshold based on your line color

# Main loop for line following
while True:
    # Get RGB values from the color sensor
    r, g, b = color_sensor.rgb()

    # Check if we are on the line (dark surface)
    if r < THRESHOLD and g < THRESHOLD and b < THRESHOLD:
        # If on the line, move forward
        steering_drive.on(0, 30)  # Go straight
        ev3.leds.set_color('green')  # Set both LEDs to green
    else:
        # If off the line, stop the motors
        steering_drive.off()

        # Determine which way to turn based on RGB values
        if r > g and r > b:
            # More red detected (off line)
            steering_drive.on(100, 0)  # Turn right
            ev3.leds.set_color('blue')  # Set left LED to blue, right LED to green
        else:
            # More green or blue detected (off line)
            steering_drive.on(-100, 0)  # Turn left
            ev3.leds.set_color('green')  # Set left LED to green, right LED to blue

    wait(10)  # Small delay to prevent rapid polling
