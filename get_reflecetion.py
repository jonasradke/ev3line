#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import ColorSensor
from pybricks.tools import wait
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port

# Initialize the EV3 brick and color sensor
ev3 = EV3Brick()
color_sensor = ColorSensor(Port.S2)  # Make sure your color sensor is connected to Port S2
ev3.light.off()

while True:
    # Read the reflection level from the color sensor
    reflection = color_sensor.reflection()

    # Clear the screen before updating
    ev3.screen.clear()
    # Display the reflection value on the EV3 screen
    ev3.screen.print("Reflection:", reflection)
    print("Reflection:", reflection)

    # Wait a bit before the next reading to make the output readable
    wait(100)  # 100 milliseconds delay
