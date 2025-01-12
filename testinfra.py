#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import UltrasonicSensor
from pybricks.parameters import Port
from pybricks.hubs import EV3Brick
from pybricks.tools import wait

ev3 = EV3Brick()

# Initialize the sensors. test
ir_sensor = UltrasonicSensor(Port.S1)



while True:
    ev3.screen.clear()
    ev3.screen.print(ir_sensor.distance())
    print(ir_sensor.distance())
    wait(100)
