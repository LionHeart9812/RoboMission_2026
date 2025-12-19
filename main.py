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
from linefollower import DriveTillDouble, DriveTillColor, LineFollower_tillDouble

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
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=199)

# Einstellungen variable("robot")
robot.settings(600, 550, 200, 150)

def fahren(speeder):
    robot.drive(speeder/7, 0)
    time.sleep(0.25)
    robot.drive(speeder/5, 0)
    time.sleep(0.25)
    robot.drive(speeder/2, 0)
    time.sleep(0.25)
    robot.drive(speeder, 0)

# Auf: positiv; Zu: negativ
def grabben(speeder, timerMS):
    grabber_left.run(speeder)
    grabber_right.run(speeder)
    wait(timerMS)
    grabber_left.stop()
    grabber_right.stop()
            
def RitterErkennen():
    start = time.time()

    while time.time() - start < 2.75:
        try:
            nr_blocks, blocks = pixy.get_blocks(25, 1)
        except OSError as e:
            print("Pixy Fehler:", e)
            # kleine Pause und nächste Runde probieren
            time.sleep(0.1)
            continue

        print("nr_blocks:", nr_blocks)
        if nr_blocks >= 1:
            colorSIG = blocks[0].sig
            print("Signature of Color:", colorSIG)
            if colorSIG == 1:
                return "blue"
            elif colorSIG == 2:
                return "yellow"
            elif colorSIG == 3:
                return "green"
            elif colorSIG == 4:
                return "red"
            elif colorSIG == 5:
                return "white"
            
        time.sleep(0.2)

    # Wenn in der Zeitspanne nichts Passendes erkannt wurde
    return None
            
def checkUp():
    robot.straight(-100)
    wait(100)
    robot.turn(90)
    wait(250)
    robot.turn(90)
    wait(250)
    robot.turn(90)
    wait(250)
    robot.turn(90)
    wait(250)
    robot.straight(90)
    time.sleep(100)

##---------------------------- Fahrprogramm ----------------------------##
## Anfahren ##
# robot.straight(1100)
# robot.turn(-90)
# fahren(-1000)
# wait(200)
# robot.stop()
# DriveTillDouble(7, 400)
detectedColor = RitterErkennen()
print(detectedColor)