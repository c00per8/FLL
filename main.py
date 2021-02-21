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
AXLE_TRACK = 135

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
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = -170

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2

DISTANCE_TO_ROLLERS = (36+54+12+30)*10
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

def base_to_basket():
    global robot, DISTANCE_TO_BASKET
    drive_speed = -100
    porportional_gain = 1.2
    robot.straight(-100)
    lineFollow(DISTANCE_TO_BASKET, drive_speed, porportional_gain)
    robot.turn(-90)
    lineFollow(150, drive_speed, porportional_gain)

def lift_basket():
    TURN_ANGLE=24
    print("robot settings(straight speed, straight accel, turn_rate, turn_accel):" + str(robot.settings()))
    robot.settings(straight_acceleration=100, turn_acceleration=100)

    #remove if needed
    robot.straight(-200)
    robot.turn(180)

    # robot.straight(-500)
    # robot.turn(-10)
    # robot.straight(-120)
    # robot.turn(18)
    # robot.straight(-200)
    # robot.straight(10)
    # small_motor.control.stall_tolerances(speed=15, time=1000)
    # print("control limits (speed, acceleration, actuation):" + str(small_motor.control.limits()))
    # print("stall tolerances(speed, time):" + str(small_motor.control.stall_tolerances()))

    #lift basket
    rotated = 0
    small_motor.reset_angle(angle=0)
    while rotated <= 600:
        small_motor.run_time(speed=500, time=1000, then=Stop.COAST, wait=False)
        rotated = small_motor.angle()
        if small_motor.control.stalled():
            print("stalled")
            print("rotated:" + str(rotated))
            small_motor.run_time(speed=-1000, time=500, then=Stop.COAST, wait=True)
            rotated = small_motor.angle()
            print("rotated:" + str(rotated))

    rotated = small_motor.angle()
    print("final rotated:" + str(rotated))
    #small_motor.run_time(speed=-1000, time=600, then=Stop.COAST, wait=True)
    
    #reverse to mission 08
    robot.straight(50)
    robot.turn(15)
    robot.straight(150)
    robot.turn(-40)

    # lower the orange boom for mission 8
    while rotated > 400:
        small_motor.run_time(speed=-1000, time=100, then=Stop.COAST, wait=True)
        rotated = small_motor.angle()

    # GOTO mission 08
    robot.straight(-230)

    # lift the orange boom
    while rotated < 480:
        small_motor.run_time(speed=1000, time=100, then=Stop.COAST, wait=True)
        rotated = small_motor.angle()

    #TODO: turn and reverse to return home
    robot.turn(-55)
    robot.straight(6000)

def mission_treadmill():
    global robot
    porportional_gain = 0.8
    drive_speed = -170
    # drive straight for 10cm
    robot.straight(-100)
    
    # go to rollers
    print("will stop in {}mm".format(DISTANCE_TO_ROLLERS))
    lineFollow(DISTANCE_TO_ROLLERS-50, drive_speed, porportional_gain)
    robot.straight(-75)

    #Run treadmill
    small_motor.run_time(-500, 1000)
    small_motor.run_time(500, 11000)
    
    robot.turn(90)
    lineFollow(130, drive_speed, porportional_gain)
    robot.turn(180)
    lineFollow(130, drive_speed, porportional_gain)
    robot.turn(-90)

    #turn around and return to base
    robot.turn(angle=180)
    lineFollow(DISTANCE_TO_ROLLERS, drive_speed, porportional_gain)



def mission_boccia():
    global robot
    DISTANCE_TO_FIRST_TURN = 780
    DISTANCE_TO_BOCCIA = 590
    drive_speed = -110
    porportional_gain = 1.1
    
    # drive straight for 10cm
    robot.straight(-50)
    lineFollow(DISTANCE_TO_FIRST_TURN, drive_speed, porportional_gain)
    robot.turn(90)
    lineFollow(DISTANCE_TO_BOCCIA, drive_speed, porportional_gain)
    for i in range(9):
        robot.turn(-10)
        time.sleep(0.1)
    
def mission_boccia2():
    global robot
    DISTANCE_TO_FIRST_TURN = 820
    DISTANCE_TO_BOCCIA = 615
    drive_speed = -100
    porportional_gain = 1.1
    robot.turn(90)
    lineFollow(DISTANCE_TO_BOCCIA, drive_speed, porportional_gain)
    for i in range(9):
        robot.turn(-10)
        time.sleep(0.1) 

def mission_stepcounter():
    global robot
    robot.settings(straight_speed=20)
    robot.straight(-1000)


def main():
    #mission_boccia()
    #mission_boccia2()
    #mission_treadmill()
    #mission_stepcounter()
    # small_motor.run_time(-500, 1000)
    # small_motor.run_time(500, 9000)
    #TODO: add other missions
    #base_to_basket()
    lift_basket()
    
    #lineFollow(600,-100,1.1)
   #small_motor.run_time(500, 11000)


if __name__ == "__main__":
    main()

#    small_motor.run_target(200, 670) 