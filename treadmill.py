#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time 

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
small_motor = Motor(Port.C)

# # Write your program here.
ev3.speaker.beep()

WHEEL_SIZE = 93.5
AXLE_TRACK = 112

#TODO: Test Line follower code

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S1) 





# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_SIZE, axle_track=AXLE_TRACK)

#To calibrate color sensors. 
amb = line_sensor.ambient()
print("ambient is:" + str(amb))
ref = line_sensor.reflection()
print("reflection is:" + str(ref))
col = line_sensor.color()
print("col is:" + str(col))


# Calculate the light threshold. Choose values based on your measurements.
BLACK = 8
WHITE = 95

# outside in the garage, white is 75
WHITE = 75
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = -170

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.1

DISTANCE_TO_ROLLERS = (36+54+12+30)*10

#TEST distance
#DISTANCE_TO_ROLLERS = (66)*10

DISTANCE_TO_BASKET = (25+64)*10

def lineFollow(travel_distance, drive_speed, porportional_gain):
    global robot
    robot.reset()

    while True:

        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = porportional_gain * deviation

        # Set the drive base speed and turn rate.
        robot.drive(drive_speed, turn_rate)
        print("distance traveled: "+str(abs(robot.distance())))
        if abs(robot.distance()) >= travel_distance:
            robot.stop()
            break
        # You can wait for a short time or do other things in this loop.
        wait(10)

porportional_gain = 0.8
drive_speed = -170
# Drive straight for 10cm
robot.straight(-150)
    
# go to rollers
print("will stop in {}mm".format(DISTANCE_TO_ROLLERS))
lineFollow(DISTANCE_TO_ROLLERS-50, drive_speed, porportional_gain)
robot.straight(-65)

    #Run treadmill
small_motor.run_time(500, 4000)
    
# robot.turn(90)
# lineFollow(130, drive_speed, porportional_gain)
# robot.turn(180)
# lineFollow(130, drive_speed, porportional_gain)
# robot.turn(-90)

#     #turn around and return to base
# robot.turn(angle=180)
# lineFollow(DISTANCE_TO_ROLLERS, drive_speed, porportional_gain)


