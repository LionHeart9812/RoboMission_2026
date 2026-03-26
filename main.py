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

# Create your objects here./Studiengang: Türme erichten
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

#Gestell des Roboters (Durchmessser der Reifen, Radstand)-hahaha
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=199)
robot.settings(750, 400, 450, 350)

# Fahren ohne Distanz
def fahren(speeder):
    if speeder >= 200 or speeder < -200:
        robot.drive(speeder/3, 0)
        time.sleep(.5)

    robot.drive(speeder/1.5, 0)
    time.sleep(.75)
    robot.drive(speeder, 0)
            
# ------------------ Main programm ------------------ #
# Warten bis Mittelknopf gedrückt
ev3.screen.draw_text(x=10, y=10, text="Ready to start")
print("Programm is ready")
while Button.CENTER not in ev3.buttons.pressed():
    pass
ev3.screen.clear()
ev3.speaker.beep()
wait(200)

# --- Blau + Schwarze Mensch nehmen --- #
lifter.run_time(-200, 500, then=Stop.HOLD, wait=False)
grabber.run_time(-750, 1750, then=Stop.HOLD, wait=False)
robot.straight(315)
robot.turn(-35)
robot.straight(140)
lifter.run_time(200, 350, then=Stop.COAST, wait=False)
robot.straight(-5)
robot.turn(-90)
DriveTillColor("left", 7, 400)
robot.straight(20)
robot.stop()

robot.turn(40)
robot.straight(480)
robot.stop()

# Dreckklumpen mitnehmen
LF_StopBlack(2.5, 0.4, 25, 300)
robot.turn(25)
robot.stop()
motor_right.run_angle(500, 500)
motor_right.stop()

robot.straight(215)
grabber.run_time(-750, 1500)
lifter.run_time(-200, 750, then=Stop.HOLD, wait=False)

robot.straight(-100)
robot.stop()
robot.turn(35)
grabber.run_time(750, 1250)
robot.straight(100)
lifter.run_time(200, 250, then=Stop.COAST)

grabber.run_time(750, 1700, then=Stop.HOLD)
lifter.run_time(-175, 700, then=Stop.HOLD)
robot.straight(-200)

robot.stop()
motor_left.run_angle(500, 500)
motor_left.stop()
#robot.turn(80)
robot.straight(225)

time.sleep(60)
# --- Türme stacken --- #
# grabber.run_time(400, 5000, then=Stop.HOLD, wait=False)
lifter.run_time(-200, 300, then=Stop.HOLD, wait=False)
robot.straight(195)
robot.turn(94)
DriveTillDouble(7, 250)

# In beide Türme reinfahren
robot.straight(170)
robot.straight(-20)
lifter.run_time(200, 75, then=Stop.COAST, wait=False)
grabber.run_time(-750, 3200, then=Stop.HOLD)

robot.straight(-60)
DriveTillDouble(7, -200)
robot.stop()
robot.turn(93)
robot.straight(200)
DriveTillDouble(7, 450)

robot.turn(70)
DriveTillColor("left", 7, 300)
robot.stop()

# Bro steht bei der Mittellinie
robot.straight(-10)
robot.turn(-58)
robot.straight(125)
robot.stop()
LF_StopBlack(2.5, 0.4, 25, 300)
robot.straight(-20)
grabber.run_time(700, 3800, wait=False)
wait(3000)

### Rechten Turm abstellen
lifter.run_time(-200, 150, then=Stop.HOLD)
robot.straight(-150)
robot.straight(-10)
grabber.run_angle(-700, 2750, wait=False)
robot.turn(91)
robot.straight(100)
robot.turn(-90)
robot.straight(180)

grabber.run_time(700, 4100, then=Stop.HOLD)
lifter.run_time(-150, 1000, then=Stop.HOLD)

robot.straight(-85)
robot.turn(90)
robot.straight(155)

# Abstellen
lifter.run_time(150, 325, then=Stop.HOLD)
grabber.run_time(-750, 3000, then=Stop.HOLD)

grabber.run_time(-750, 750, then=Stop.HOLD, wait=False)
robot.straight(-200)

### Den linken Turm wegbringen
lifter.run_time(150, 400, then=Stop.COAST, wait=False)

robot.straight(-160)
robot.turn(-90)
robot.straight(175)
robot.straight(-10)

grabber.run_time(600, 4500, then=Stop.HOLD)
lifter.run_time(-150, 1000, then=Stop.HOLD)

robot.straight(-150)
robot.turn(-90)
robot.straight(142.5)

#Abstellen
lifter.run_time(150, 350, then=Stop.HOLD)
grabber.run_time(-750, 2000, then=Stop.HOLD)

robot.straight(-200)
grabber.run_time(750, 1250, then=Stop.HOLD)