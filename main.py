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
from linefollower import *

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

motor_left = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
motor_right = Motor(Port.A, positive_direction=Direction.CLOCKWISE)

lifter = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
grabber = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)

colorSensor_left = ColorSensor(Port.S1)
colorSensor_right = ColorSensor(Port.S4)

colorReflection_left =  colorSensor_left.reflection()
colorReflection_right = colorSensor_right.reflection()

pixy = Pixy2(port=2, i2c_address=0x54)

seenBlack = 0

ev3.speaker.beep()

#Gestell des Roboters (Durchmessser der Reifen, Radstand)
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=198)
robot.settings(500, 375, 250, 125)

# Fahren ohne Distanz
def fahren(speeder):
    if speeder >= 200:
        robot.drive(speeder/3, 0)
        time.sleep(.5)

    robot.drive(speeder/1.5, 0)
    time.sleep(.75)
    robot.drive(speeder, 0)

# Normale Werte für KP und KD
# LF_StopBlack(2.5, 0.2, 8)
            
# ------------------ Main programm ------------------ #
# --- Türme stacken --- #
robot.straight(140)
DriveTillColor("left", 7, 150)
robot.straight(25)
robot.turn(-90)
DriveTillColor("right", 7, 150)
robot.straight(35)

robot.turn(92)
grabber.run_angle(-250, 280)
robot.straight(155)
robot.straight(-10)
grabber.run_time(250, 650, then=Stop.HOLD)

robot.straight(-60)
DriveTillDouble(7, -100)
robot.turn(94)
robot.straight(200)
DriveTillDouble(7, 400)

robot.straight(50)
robot.turn(90)
DriveTillColor("left", 7, 300)

robot.straight(-40)
robot.turn(-90)
robot.straight(125)
robot.stop()
LF_StopBlack(2.5, 0.2, 25)
grabber.run_angle(-250, 150)

# Rechten Turm abstellen
robot.straight(-150)
robot.turn(90)
robot.straight(100)
robot.turn(-90)
grabber.run_angle(250, 200)
robot.straight(150)
grabber.run_time(250, 175, then=Stop.HOLD)


# --- Fahren durch die Mitte --- #
# LF_StopBlack(2.5, 0.2, 8)
# robot.straight(350)
# robot.stop()
# wait(100)
# LF_StopBlack(2.5, 0.2, 25)