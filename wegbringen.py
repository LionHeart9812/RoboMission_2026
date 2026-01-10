#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pixycamev3.pixy2 import Pixy2
import time
from linefollower import DriveTillDouble, DriveTillColor, LineFollower_tillDouble, LineFollower_tillTime

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
motor_left = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
motor_right = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
grabber_left = Motor(Port.C, positive_direction=Direction.CLOCKWISE)
grabber_right = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
colorSensor_left = ColorSensor(Port.S1)
colorSensor_right = ColorSensor(Port.S4)
colorReflection_left =  colorSensor_left.reflection()
colorReflection_right = colorSensor_right.reflection()
pixy = Pixy2(port=2, i2c_address=0x54)

#Gestell des Roboters (Durchmessser der Reifen, Radstand)
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=199)

# Einstellungen variable("robot")
robot.settings(600, 550, 200, 150)

def fahren(speeder):
    robot.drive(speeder/2, 0)
    time.sleep(.2)
    robot.drive(speeder, 0)

# Auf: positiv; Zu: negativ
def grabben(speeder, timerS):
    grabber_left.run(speeder)
    grabber_right.run(speeder)
    time.sleep(timerS)
    grabber_left.hold()
    grabber_right.hold()

# Erstellen der funktionen
def red():
    robot.straight(140)
    robot.turn(90)
    grabben(-100, .15)
    robot.straight(110)
    grabben(-150, 1)
    robot.straight(-140)
    robot.turn(-89)
    DriveTillColor("left", 7, 250)
    time.sleep(.1)

    robot.straight(-65)
    robot.turn(-89)
    robot.straight(135)
    grabben(150, 1)

    #Zurück
    DriveTillDouble(7, -150)
    time.sleep(.1)
    robot.straight(50)
    robot.turn(90)
    fahren(-350)
    time.sleep(3)
    robot.stop()
    robot.straight(300)

def yellow():
    robot.straight(140)
    robot.turn(90)
    grabben(-100, .15)
    robot.straight(110)
    grabben(-150, 1)
    robot.straight(-120)
    robot.turn(-89)
    DriveTillColor("left", 7, 250)
    robot.stop()
    LineFollower_tillDouble()
    time.sleep(.1)

    robot.turn(-25)
    robot.straight(300)
    robot.turn(25)
    robot.straight(100)
    grabben(150, 1)
    grabber_left.stop()
    grabber_right.stop()

    #Zurück
    robot.straight(-50)
    robot.turn(-90)
    fahren(-350)
    time.sleep(1)
    robot.stop()
    time.sleep(.1)
    robot.straight(200)
    robot.turn(-90)
    time.sleep(.2)
    DriveTillDouble(7, 250)
    robot.stop()
    LineFollower_tillTime(5)
    robot.turn(-180)
    fahren(-350)
    time.sleep(2)
    robot.stop()
    robot.straight(300)



def blue():
    robot.straight(100)
    robot.turn(90)
    grabben(-100, .15)
    robot.straight(110)
    grabben(-150, 1)
    robot.straight(-120)
    robot.turn(-89)
    DriveTillColor("left", 7, 250)
    robot.turn(5)
    robot.stop()
    LineFollower_tillDouble()
    time.sleep(.1)

    robot.turn(-50)
    robot.straight(535)
    grabben(150, 1)
    grabber_left.stop()
    grabber_right.stop()

    #Zurück
    robot.turn(-40)
    robot.straight(-50)
    fahren(-350)
    time.sleep(3)
    robot.stop()
    time.sleep(.1)
    robot.straight(200)
    robot.turn(-90)
    time.sleep(.2)
    DriveTillDouble(7, 250)
    robot.stop()
    LineFollower_tillTime(5)
    robot.turn(-180)
    fahren(-350)
    time.sleep(2)
    robot.stop()
    robot.straight(300)

def green():
    robot.straight(140)
    robot.turn(90)
    grabben(-100, .15)
    robot.straight(110)
    grabben(-150, 1)
    robot.straight(-120)
    robot.turn(-90)

    fahren(-350)
    time.sleep(3)
    robot.stop()
    robot.straight(225)
    robot.turn(-90)
    time.sleep(.1)
    LineFollower_tillDouble()
    time.sleep(.1)

    robot.turn(-90)
    robot.straight(100)
    grabben(150, 1)
    grabber_left.stop()
    grabber_right.stop()

    #Zurück
    robot.straight(-100)
    robot.turn(90)
    fahren(-350)
    time.sleep(2.5)
    robot.stop()
    robot.straight(120)
    robot.turn(90)