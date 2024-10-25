#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from pybricks.hubs import EV3Brick

# Initialize the EV3 brick and color sensor
ev3 = EV3Brick()
color_sensor = ColorSensor(Port.S2)  # Make sure your color sensor is connected to Port S3
ev3.light.off()

while True:
    # Read the RGB values from the color sensor
    rgb = color_sensor.rgb()
    
    # Clear the screen before updating
    ev3.screen.clear()
    ev3.screen.print(rgb[0],rgb[2])
    # Display the RGB values on the EV3 screen

    # Wait a bit before the next reading to make the output readable
    wait(500)  # 500 milliseconds delay
