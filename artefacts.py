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
#
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
perfect = {}
perfectColor = {}
goalPlace = {
    "red": 0,
    "green": 1,
    "black": 2,
    "blue": 3,
    "yellow": 4
}

# --- Funktionen --- #
def zweiGrabben():
    robot.drive(350, 0)
    wait(600)
    robot.stop()
    grabber.run_time(-750, 2000)
    grabber.stop()
    robot.straight(-675)

def vertauschtRechts():
    grabber.run_time(750, 2200, then=Stop.HOLD)
    grabber.stop()
    robot.straight(50)
    robot.stop()
    motor_right.run_angle(450, 350)
    motor_right.run_angle(-450, 500)
    motor_right.stop()
    robot.straight(150)
    robot.straight(-150)

def vertauschtLinks():
    grabber.run_time(750, 2200, then=Stop.HOLD)
    grabber.stop()
    robot.straight(50)
    robot.stop()
    motor_left.run_angle(450, 350)
    motor_left.run_angle(-450, 500)
    motor_left.stop()
    robot.straight(150)
    robot.straight(-150)
    
def scan():
    nr_blocks, blocks = pixy.get_blocks(0x1F, 3)
    minWidth = 70

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
    correctOrientationExists = False

    for i in range(len(pos) - 1):
        a = pos[i]
        b = pos[i + 1]

        if abs(goalPlace[a] - goalPlace[b]) == 1:
            if goalPlace[a] - goalPlace[b] == 1:
                print("Richtige Artefaktposition")
                correctOrientation = True
                correctOrientationExists = True

            elif goalPlace[a] - goalPlace[b] == -1:
                print("Getauschte Artefaktposition")
                correctOrientation = False

            perfect[(pos.index(a), pos.index(b))] = correctOrientation
            perfectColor[(a, b)] = correctOrientation
            two_artefacts = True
    
    print("perfectColor:", perfectColor)
    print(perfect)
    print("------------")
    return two_artefacts, correctOrientationExists

# Checken, welche Prios
def checkPrio():
    isTwoArtefacts, correctOrentation = twoArtefacts()
    whichIndex = -1

    if isTwoArtefacts:
        if correctOrentation:
            priority = 4
        else:
            priority = 3

    elif not isTwoArtefacts:
        for i in pos:
            if i == "yellow" or i == "red":
                priority = 2
                whichIndex = pos.index(i)
            else:
                priority = 1
                whichIndex = pos.index(i)

    return priority, whichIndex

# Wegbringen
def artefacts(prio, OutsiderIndex):
    pixy.set_lamp(0, 0)

    # Check Priority and give positioning
    # if there is a correctOrentiation one
    if prio == 4:
        for pairs, orientValue in perfect.items():
            if not orientValue:
                print("not the right one")
                pass

            elif orientValue:
                print("the right one")
                print(pairs)
                print(0 in pairs)
                if 0 in pairs:
                    postioning = ("right", "true")
                elif 3 in pairs:
                    postioning = ("left", "true")
                else:
                    postioning = ("middle", "true")
   
                break

    # if there isn't
    elif prio == 3:
        for pairs, orientValue in perfect.items():
            if 0 in pairs:
                postioning = ("right", "false")
            elif 3 in pairs:
                postioning = ("left", "false")
            else:
                postioning = ("middle", "false")
                
            break

    elif prio == 2 or prio == 1:
        if OutsiderIndex == 0:
            postioning = ("right", "outsider")
        elif OutsiderIndex == 1:
            postioning = ("middleRight", "outsider")
        elif OutsiderIndex == 2:
            postioning = ("middleLeft", "outsider")
        elif OutsiderIndex == 3:
            postioning = ("left", "outsider")
        else:
            print("Non-Existent?")

    #Drive to the places
    side, orientation = postioning
    print("Positioning:", side, orientation)
    print("---------------")
    DriveTillDouble(9, -200)

    if prio == 4 or prio == 3:
        #Checken, welche Seite Steinis ist
        if side == "right":
            robot.straight(-100)

        elif side == "left":
            robot.straight(150)
    
        else:
            robot.straight(35)

        # Nehmen
        robot.turn(90)
        zweiGrabben()

        #Checken, welche seite Guppy der süße ist
        if side == "left":
            robot.turn(90)
            DriveTillDouble(9, 200)
            robot.stop()

        elif side == "middle":
            robot.straight(10)
            robot.turn(90)
            robot.stop()

        else:
            robot.turn(90)
            DriveTillDouble(9, -200)
            robot.stop()

        # Wegbringen wenn prio 4 ist
        if prio == 4:
            for colorPairs, orientColor in perfectColor.items():
                if orientColor:
                    right, left = colorPairs
                    print("Right:", right)

            if right == "yellow":
                robot.straight(-150)
                robot.turn(92)

            elif right == "blue":
                robot.straight(-100)
                robot.turn(92)

            elif right == "black":
                robot.straight(100)
                robot.turn(92)

            elif right == "green":
                robot.straight(165)
                robot.turn(92)

            robot.straight(175)
            grabber.run_time(750, 2100, then=Stop.HOLD)
            # robot.turn(10)
            # robot.turn(-20)
            # robot.turn(10)
        
        if prio == 3:
            right, left = perfectColor[0]

            print("Left:", left)
            print("Right:", right)

            if right == "blue":
                robot.straight(180)
                robot.straight(100)
                robot.turn(92)

                if left == "yellow":
                    vertauschtRechts()
                else:
                    vertauschtLinks()

            elif right == "black":
                robot.stop(50)
                robot.turn(92)

                if left == "blue":
                    vertauschtRechts()
                else:
                    vertauschtLinks()

            elif right == "green":
                robot.straight(100)
                robot.turn(92)

                if left == "black":
                    vertauschtRechts()
                else:
                    vertauschtLinks()

            if right == "red":
                robot.straight(150)
                robot.turn(92)

                if left == "green":
                    vertauschtRechts()
                else:
                    vertauschtLinks() 

    else:
        robot.turn(-90)


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
priority, whichIndex = checkPrio()
print("Priority:", priority)
print("Index of Red/Yellow:", whichIndex)
artefacts(priority, whichIndex)
grabber.stop()
robot.stop()