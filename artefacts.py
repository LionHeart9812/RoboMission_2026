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

#Gestell des Roboters (Durchmessser der Reifen, Radstand)
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=199)

# Einstellungen variable("robot")
robot.settings(750, 400, 450, 400)

# Globale Variabels
AllArtefacts = 4

pos = []
goalPlace = {
    "red": 0,
    "green": 1,
    "black": 2,
    "blue": 3,
    "yellow": 4
}

# Funktionen
def scan():
    nr_blocks, blocks = pixy.get_blocks(0x1F, 3)

    if nr_blocks >= 1:
        for b in blocks[:nr_blocks]:
            if b.sig == 1 and b.width >= 45:
                print("Höhe des Farbe:", b.width)
                return "red"
            elif b.sig == 2 and b.width >= 45:
                print("Höhe des Farbe:", b.width)
                return "yellow"
            elif b.sig == 3 and b.width >= 45:
                print("Höhe des Farbe:", b.width)
                return "green"
            elif b.sig == 4 and b.width >= 45:
                print("Höhe des Farbe:", b.width)
                return "blue"
            else:
                print("Wahrscheinlich schwoarz")
                return "black"

def twoArtefacts():
    perfect = []
    two_artefacts = False

    for i in range(len(pos) - 1):
        a = pos[i]
        b = pos[i + 1]

        if abs(goalPlace[a] - goalPlace[b]) == 1:
            perfect.append((a, b))
            two_artefacts = True

    return perfect, two_artefacts


def checkPrio():
    pass

def artefacts():
    while not AllArtefacts == 0:
        pass

count = 3
while count > 0:
    wait(150)
    color = scan()
    print(color)

    robot.straight(125)
    pos.append(color)
    count -= 1

wait(150)
color = scan()
print(color)
pos.append(color)

print(pos)
whichBlocks, two_artefacts = twoArtefacts()
print(whichBlocks)