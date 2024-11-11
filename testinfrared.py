from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase, Stop
from pybricks.tools import wait

# Initialize the EV3 brick.
ev3 = EV3Brick()

# Initialize the sensors.
ir_sensor = InfraredSensor(Port.S1)



while True:
    print(ir_sensor.proximity)
