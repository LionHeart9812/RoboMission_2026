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
colorSensor_left = ColorSensor(Port.S1)
colorSensor_right = ColorSensor(Port.S4)
colorReflection_left =  colorSensor_left.reflection()
colorReflection_right = colorSensor_right.reflection()
pixy = Pixy2(port=2, i2c_address=0x54)

#Gestell des Roboters (Durchmessser der Reifen, Radstand)
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=199)

# Einstellungen variable("robot") + Pixy
robot.settings(750, 400, 450, 400)
pixy.set_lamp(1, 1)

# Globale Variabels
AllArtefacts = 4

pos = []
perfect = []
goalPlace = {
    "red": 0,
    "green": 1,
    "black": 2,
    "blue": 3,
    "yellow": 4
}

# --- Funktionen --- #
def scan():
    nr_blocks, blocks = pixy.get_blocks(0x1F, 3)
    minWidth = 60

    if nr_blocks >= 1:
        for b in blocks[:nr_blocks]:
            if b.sig == 1 and b.width >= minWidth:
                print("Breite der Farbe:", b.width)
                return "red"
            elif b.sig == 2 and b.width >= minWidth:
                print("Breite der Farbe:", b.width)
                return "yellow"
            elif b.sig == 3 and b.width >= minWidth:
                print("Breite der Farbe:", b.width)
                return "green"
            elif b.sig == 4 and b.width >= minWidth:
                print("Breite der Farbe:", b.width)
                return "blue"
            else:
                print("Wahrscheinlich schwoarz")
                return "black"
            
    else:
        print("Wahrscheinlich schwoarz")
        return "black"

# Scannen    
def scanDrive():
    count = AllArtefacts - 1
    while count > 0:
        color = scan()
        print(color)

        robot.straight(125)
        pos.append(color)
        count -= 1

    color = scan()
    print(color)
    pos.append(color)

    print("----------------")
    print(pos)
    print("----------------")

# Checken, ob es zwei nebeneinander gibt
def twoArtefacts():
    two_artefacts = False
    correctOrientation = False

    for i in range(len(pos) - 1):
        a = pos[i]
        b = pos[i + 1]

        if abs(goalPlace[a] - goalPlace[b]) == 1:
            if goalPlace[a] - goalPlace[b] == 1:
                print("Richtige Artefaktposition")
                correctOrientation = True

            elif goalPlace[a] - goalPlace[b] == -1:
                print("Getauschte Artefaktposition")

            perfect.append(pos.index(a))
            perfect.append(pos.index(b))
            two_artefacts = True
    
    print(perfect)
    return two_artefacts, correctOrientation

# Checken, welche Prios
def checkPrio():
    isTwoArtefacts, correctOrentation = twoArtefacts()

    if isTwoArtefacts:
        if correctOrentation:
            priority = 4
        else:
            priority = 3

    elif not isTwoArtefacts:
        for i in pos:
            if i == "yellow" or i == "red":
                priority = 2
            else:
                priority = 1

    return priority

# Wegbringen
def artefacts(prio):
    pixy.set_lamp(0, 0)

    # Check Priority and give positioning
    if prio == 4:
        if 0 in perfect:
            postioning = ("right", "outside", "correct")
        elif 3 in perfect:
            postioning = ("left", "outside", "correct")
        else:
            postioning = ("middle", "inside", "correct")

    elif prio == 3:
        if 0 in perfect:
            postioning = ("right", "outside", "false")
        elif 3 in perfect:
            postioning = ("left", "outside", "false")
        else:
            postioning = ("middle", "inside", "false")

    #Drive to the places
    side, where, orientation = postioning
    DriveTillDouble(9, -200)

    if side == "right":
        robot.straight(-100)

    elif side == "left":
        robot.straight(100)
    
    else:
        robot.straight(35)

    robot.turn(90)


# ---- Main Programm ---- #
# Warten bis Mittelknopf gedrückt
ev3.screen.draw_text(x=10, y=10, text="Ready to start")
print("Programm is ready")
ev3.light.on(Color.ORANGE)
while Button.CENTER not in ev3.buttons.pressed():
    pass
ev3.screen.clear()
ev3.light.on(Color.GREEN)
ev3.speaker.beep()
wait(200)

# while AllArtefacts < 0:

# Scannen
scanDrive()
priority = checkPrio()
print(priority)
artefacts(priority)