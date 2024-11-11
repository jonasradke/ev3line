#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase, Stop
from pybricks.tools import wait, StopWatch 

# Initialize the stopwatch
stopwatch = StopWatch()


# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
drop_motor = Motor(Port.A)

# Initialize the EV3 brick.
ev3 = EV3Brick()

# Initialize the sensors.
line_sensor = ColorSensor(Port.S2)
#ir_sensor = InfraredSensor(Port.S1)



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
INTEGRAL_GAIN = 0.07
DERIVATIVE_GAIN = 0.2

# Initialize variables for the PID controller
integral = 0
last_error = 0
blue_detected = False
blue_detected2 = True
block_detected = False

#while not ev3.buttons.pressed():
        #wait(10)


# Start following the line endlessly.
while True:
    ev3.screen.clear()
    ev3.screen.print(stopwatch.time())


    # Get the current RGB value from the sensor
    current_rgb = line_sensor.rgb()

    # Compare the current RGB value with the calibrated blue RGB values
    #detect blue and turn
    if (abs(current_rgb[0] < 10) and 
        abs(current_rgb[1] < 15) and 
        abs(current_rgb[2] > 30)) and not blue_detected:
        # If the color is close enough to blue, perform actions like moving forward and turning
            robot.straight(20)
            robot.turn(50)
            integral = 0
            blue_detected = True  # Set flag to prevent multiple triggers        
            
    #detect blue and run straight
    if (abs(current_rgb[0] < 10) and 
        abs(current_rgb[1] < 15) and 
        abs(current_rgb[2] > 30)) and not blue_detected2:
            print("blue detected")
            robot.straight(100)
            blue_detected2 = True


    #detect red and drop the object
    if (abs(current_rgb[0] > 27) and 
        abs(current_rgb[1] < 13) and 
        abs(current_rgb[2] < 20)) and blue_detected:
            print("red detected")
            drop_motor.run_angle(1000, 180, then=Stop.HOLD, wait=False) 
            
        
    #detect green and do stuff
    if (abs(current_rgb[0] < 14) and 
        abs(current_rgb[1] > 20) and 
        abs(current_rgb[2] < 20)):
            print("green detected")
            robot.straight(50)
            drop_motor.run_angle(1000, -180, then=Stop.HOLD, wait=False) 
            blue_detected2 = False
            


    #if (ir_sensor.proximity < 30) and not block_detected: #noah
        # If the Block is near the robot, prints out a message
        #sound.speak('Hindernis erkannt')
        #block_detected = True

    # Get the current light reflection and calculate the deviation from the threshold.
    light_value = line_sensor.reflection()
    error = light_value - threshold

    # Update integral and derivative terms
    integral += error
    derivative = error - last_error

    # Calculate turn rate using the PID formula
    turn_rate = (PROPORTIONAL_GAIN * error) + (INTEGRAL_GAIN * integral) + (DERIVATIVE_GAIN * derivative)

    # Set the drive base speed and turn rate.
    #robot.drive(DRIVE_SPEED, turn_rate)

    # Update the last error for the next loop iteration
    last_error = error
