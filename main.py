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

seenBlack = 0
i = 1
lastDirection = "none"

# Write your program here.
ev3.speaker.beep()

#Gestell des Roboters (Durchmessser der Reifen, Radstand)
# Bens Werte: Wheel diameter 60; axle Track 183
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=199)

# Einstellungen variable("robot")
robot.settings(600, 550, 200, 150)

def fahren(speeder):
    robot.drive(speeder/2, 0)
    time.sleep(.2)
    robot.drive(speeder, 0)

# Auf: positiv; Zu: negativ
def grabben(speeder, timerMS):
    grabber_left.run(speeder)
    grabber_right.run(speeder)
    wait(timerMS)
    grabber_left.hold()
    grabber_right.hold()
            
# Test
grabben(150, 1500)