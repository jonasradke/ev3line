#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase, Stop
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import Font
import _thread

# Define fonts
big_font = Font(size=48, bold=True)

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
drop_motor = Motor(Port.D)

# Initialize the EV3 brick.
ev3 = EV3Brick()

# Initialize the sensors.
line_sensor = ColorSensor(Port.S2)
us_sensor = UltrasonicSensor(Port.S3)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Calculate the light threshold.
BLACK = 6
WHITE = 46
threshold = (BLACK + WHITE) / 2

# Set the drive speed.
DRIVE_SPEED = 100

# PID constants
PROPORTIONAL_GAIN = 2.5
INTEGRAL_GAIN = 0.05
DERIVATIVE_GAIN = 0.6

# Initialize variables for the PID controller
integral = 0
last_error = 0
blue_detected = False
blue_detected2 = False
block_detected = False
lap = 0

"""while not ev3.buttons.pressed():
        wait(10)

# Display the countdown on the EV3 screen
ev3.screen.set_font(big_font)
ev3.speaker.set_speech_options(language='en', voice='f2', speed=200)
ev3.screen.clear()
ev3.screen.draw_text(80, 64, "3", )
#ev3.speaker.say("Three")
ev3.light.on(Color.RED)
wait(1000)
ev3.screen.clear()
ev3.screen.draw_text(80, 64, "2")
#ev3.speaker.say("Two")
wait(1000)
ev3.screen.clear()
ev3.screen.draw_text(80, 64, "1")
#ev3.speaker.say("One")
ev3.light.on(Color.ORANGE)
wait(1000)
ev3.screen.clear()
ev3.screen.draw_text(80, 64, "0")
#ev3.speaker.say("Go")
ev3.light.on(Color.GREEN)
wait(1000)"""

# Reset the stopwatch
stopwatch = StopWatch()

# Start following the line endlessly.
while True:
   
    # Get the elapsed time in milliseconds
    elapsed_time = stopwatch.time()

    # Calculate minutes, seconds, and milliseconds
    minutes = elapsed_time // 60000
    seconds = (elapsed_time % 60000) // 1000
    milliseconds = elapsed_time % 1000

    # Format the time as mm:ss:ms using str.format()
    formatted_time = "{}:{:02}:{:03}".format(minutes, seconds, milliseconds)

    # Display it on the EV3 screen
    ev3.screen.clear()
    ev3.screen.draw_text(30, 64, formatted_time)


    # Get the current RGB value from the sensor
    current_rgb = line_sensor.rgb()

    # Compare the current RGB value with the calibrated blue RGB values

    #detect blue
    if  (lap == 1 and
        abs(current_rgb[0] < 10) and 
        abs(current_rgb[1] < 15) and 
        abs(current_rgb[2] > 22)):
        # If the color is close enough to blue, perform actions like moving forward and turning
        robot.straight(20)
        robot.turn(30)
        integral = 0

    #detect red and drop the object
    if (lap == 1 and
        abs(current_rgb[0] > 27) and 
        abs(current_rgb[1] < 13) and 
        abs(current_rgb[2] < 20)) and block_detected:
            print("red detected")
            drop_motor.run_angle(1000, 100, then=Stop.HOLD, wait=False)
             
        

    #detect green and count laps
    if (abs(current_rgb[0] < 14) and 
        abs(current_rgb[1] > 20) and 
        abs(current_rgb[2] < 20)):
            print("lap++")
            lap += 1
            wait(500)
            #ev3.speaker.say(str(lap))            
            

    if (lap == 1 and
        us_sensor.distance() < 200) and not block_detected: 
        print("block detected")
        block_detected = True

    if lap == 3:
        stopwatch.pause()
    
    if  (lap == 3 and
        abs(current_rgb[0] < 10) and 
        abs(current_rgb[1] < 15) and 
        abs(current_rgb[2] > 22)):
        # If the color is close enough to blue, perform actions like moving forward and turning
        robot.straight(20)
        robot.turn(30)
        integral = 0

    if (lap == 3 and
        abs(current_rgb[0] > 27) and 
        abs(current_rgb[1] < 13) and 
        abs(current_rgb[2] < 20)):
            robot.stop()
            

    


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
