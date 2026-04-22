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

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()
motor_left = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
motor_right = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
colorSensor_left = ColorSensor(Port.S1)
colorSensor_right = ColorSensor(Port.S4)
colorReflection_left =  colorSensor_left.reflection()
colorReflection_right = colorSensor_right.reflection()
pixy = Pixy2(port=2, i2c_address=0x54)

i = 1
lastDirection = "none"

#Gestell des Roboters (Durchmessser der Reifen, Radstand)
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=199)

# Einstellungen variable("robot")
robot.settings(750, 400, 450, 400)

def DriveTillDouble(colorIndex, speed):
    robot.drive(speed, 0)
    
    while True:
        colorReflection_left = colorSensor_left.reflection()
        colorReflection_right = colorSensor_right.reflection()
        
        if (colorReflection_left < colorIndex + 7 and colorReflection_left > colorIndex - 7 and 
            colorReflection_right < colorIndex + 7 and colorReflection_right > colorIndex - 7):
            print("color sensor left: " + str(colorReflection_left))
            print("color sensor right: " + str(colorReflection_right))
            print("----------------------------")
            break
        
    robot.stop()
    wait(100)

def DriveTillColor(whichSensor, colorIndex, speed):
    if whichSensor == "left":
        robot.drive(speed, 0)

        while True:
            colorReflection_left =  colorSensor_left.reflection()
            # Einfach gerade aus fahren mit variable speed
            # Wenn ein ausgewählter (whichsensor) Farbsenor eine bestimmte Farbe (colorIndex) sieht, dann stoppen
            if colorReflection_left < colorIndex + 3 and colorReflection_left > colorIndex - 3:
                print("color sensor left: " + str(colorReflection_left))
                print("----------------------------")
                break
        
        robot.stop()
        wait(100)

    elif whichSensor == "right":
        robot.drive(speed, 0)

        while True:
            colorReflection_right =  colorSensor_right.reflection()
            # Einfach gerade aus fahren mit variable speed
            # Wenn ein ausgewählter (whichsensor) Farbsenor eine bestimmte Farbe (colorIndex) sieht, dann stoppen
            if colorReflection_right < colorIndex + 3 and colorReflection_right > colorIndex - 3:
                print("color sensor left: " + str(colorReflection_right))
                print("----------------------------")
                break
        
        robot.stop()
        wait(100)

# Liniefolger sehr Krass
def LF_StopLine(correctionStrength, correctionRemember, colorIndex, speed):
    last_error = 0
    speed_left = 0
    speed_right = 0
    left_ref = colorSensor_left.reflection()
    right_ref = colorSensor_right.reflection()

    LINE_SPEED = speed # Basic Geschwindigkeit
    TURN_SPEED = 800 # Maximale Drehgeschwindigkeit


    while True:
        left_ref = colorSensor_left.reflection()
        right_ref = colorSensor_right.reflection()
        if left_ref <= colorIndex and right_ref <= colorIndex:
            motor_left.stop(Stop.BRAKE)
            motor_right.stop(Stop.BRAKE)
            break

        # Line following logic
        error = left_ref - right_ref
        derivative = error - last_error
        correction = correctionStrength * error + correctionRemember * derivative
        last_error = error

        speed_left = max(min(LINE_SPEED + correction, TURN_SPEED), -TURN_SPEED)
        speed_right = max(min(LINE_SPEED - correction, TURN_SPEED), -TURN_SPEED)

        motor_left.run(speed_left)
        motor_right.run(speed_right)
        wait(5)
        
    motor_left.run(0)
    motor_right.run(0)
    wait(10)
    motor_left.stop()
    motor_right.stop()
    ev3.speaker.beep()

# Liniefolger sehr Krass, aber auf Zeit digga
def LF_StopLine_Time(correctionStrength, correctionRemember, colorIndex, speed, time_ms):
    stopwatch = StopWatch()

    last_error = 0
    speed_left = 0
    speed_right = 0
    left_ref = colorSensor_left.reflection()
    right_ref = colorSensor_right.reflection()

    LINE_SPEED = speed # Basic Geschwindigkeit
    TURN_SPEED = 800 # Maximale Drehgeschwindigkeit

    stopwatch.reset()

    while stopwatch.time() < time_ms:
        left_ref = colorSensor_left.reflection()
        right_ref = colorSensor_right.reflection()
        if left_ref <= colorIndex and right_ref <= colorIndex:
            motor_left.stop(Stop.BRAKE)
            motor_right.stop(Stop.BRAKE)
            break

        # Line following logic
        error = left_ref - right_ref
        derivative = error - last_error
        correction = correctionStrength * error + correctionRemember * derivative
        last_error = error

        speed_left = max(min(LINE_SPEED + correction, TURN_SPEED), -TURN_SPEED)
        speed_right = max(min(LINE_SPEED - correction, TURN_SPEED), -TURN_SPEED)

        motor_left.run(speed_left)
        motor_right.run(speed_right)
        wait(5)
        
    motor_left.run(0)
    motor_right.run(0)
    wait(10)
    motor_left.stop()
    motor_right.stop()
    print("Zeit:", stopwatch.time())
    ev3.speaker.beep()