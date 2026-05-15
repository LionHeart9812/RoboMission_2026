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

#Gestell des Roboters (Durchmessser der Reifen, Radstand)
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=199)

# Einstellungen variable("robot")
robot.settings(750, 400, 450, 400)

# Globale Variabels
AllArtefacts = 4

pos = []
perfect = {}
goalPlace = {
    "red": 0,
    "green": 1,
    "black": 2,
    "blue": 3,
    "yellow": 4,
    "empty": 10
}

# --- Funktionen --- #
def zweiGrabben():
    robot.drive(350, 0)
    wait(600)
    robot.stop()
    grabber.run_time(-750, 2200)
    grabber.stop()
    robot.straight(-675)

def vertauschtRechts():
    grabber.run_time(750, 2050)
    grabber.stop()
    robot.straight(50)
    robot.stop()
    motor_right.run_angle(450, 330)
    motor_right.run_angle(-450, 570)
    motor_right.stop()
    robot.straight(215)
    robot.straight(-215)
    robot.stop()
    motor_right.run_angle(-450, 345)
    motor_right.stop()

def vertauschtLinks():
    grabber.run_time(750, 2050)
    grabber.stop()
    robot.straight(50)
    robot.stop()
    motor_left.run_angle(450, 330)
    motor_left.run_angle(-450, 570)
    motor_left.stop()
    robot.straight(215)
    robot.straight(-215)
    robot.stop()
    motor_left.run_angle(-450, 345)
    motor_left.stop()
    
def bandeAusrichten():
    robot.drive(-250, 0)
    ev3.speaker.beep()
    wait(1350)
    robot.stop()
    ev3.speaker.beep()
    wait(50)

    robot.straight(200)
    robot.turn(-90)

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

# Checken, ob es zwei nebeneinander gibt
def twoArtefacts():
    global perfect  
    two_artefacts = False
    correctOrientation = False
    correctOrientationExists = False
    perfect = {}

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

            perfect[(i, i+1)] = ((a, b), correctOrientation)
            two_artefacts = True
    
    print(perfect)
    print("------------")
    return two_artefacts, correctOrientationExists

# Checken, welche Prios
def checkPrio():
    global AllArtefacts
    isTwoArtefacts, correctOrentation = twoArtefacts()
    whichIndex = -1
    whichIndexColor = "None"

    if isTwoArtefacts:
        if correctOrentation:
            priority = 4
        else:
            priority = 3

        AllArtefacts -= 2

    elif not isTwoArtefacts:
        for i in pos:
            if i == "yellow" or i == "red":
                priority = 2
                whichIndex = pos.index(i)
                whichIndexColor = i
                break
            else:
                priority = 1
                if i != "empty":
                    whichIndex = pos.index(i)
                    whichIndexColor = i
                

        AllArtefacts -= 1

    return priority, whichIndex, whichIndexColor

# Wegbringen
def collect_artefacts(prio, OutsiderIndex, whichIndexColor):
    global AllArtefacts
    pixy.set_lamp(0, 0)

    # Check Priority and give positioning
    # if there is a correctOrentiation one
    if prio == 4:
        for pairs, (blockColor, orientValue) in perfect.items():
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
        for pairs, (blockIndex, orientValue) in perfect.items():
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
            robot.straight(165)
    
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
            for pairs, (blockName, orientValue) in perfect.items():
                if orientValue:
                    right, left = blockName
                    rightIndex, leftIndex = pairs
                    print("Right:", right)
                    break

            if right == "yellow":
                robot.straight(-185)
                robot.turn(92)

            elif right == "blue":
                robot.straight(-40)
                robot.turn(92)

            elif right == "black":
                robot.straight(85)
                robot.turn(92)

            elif right == "green":
                robot.straight(270)
                robot.turn(92)

            robot.drive(250, 0)
            ev3.speaker.beep()
            wait(600)
            robot.stop()
            ev3.speaker.beep()
            wait(50)
            robot.straight(-25)
            robot.stop()
            grabber.run_time(750, 2150)
            grabber.stop()

            motor_right.run_angle(450, 50)
            motor_right.run_angle(-450, 50)
            motor_right.stop()
            robot.straight(-150)
            robot.turn(90)

            # Zurückfahren        
            if right == "blue":
                robot.straight(100)

            elif right == "black":
                robot.straight(175)

            elif right == "green":
                robot.straight(250)

            robot.turn(91)
            robot.straight(525)
            robot.turn(-90)

            pos[rightIndex] = "empty"
            pos[leftIndex] = "empty"
        
        # Wegbringen wenn prio 3 ist
        if prio == 3:
            for pairs, (blockName, orientValue) in perfect.items():
                if not orientValue:
                    right, left = blockName
                    rightIndex, leftIndex = pairs
                    break

            print("Left:", left)
            print("Right:", right)
            robot.stop()

            if right == "blue":
                robot.straight(-155)
                robot.turn(90)

                if left == "yellow":
                    vertauschtRechts()
                else:
                    vertauschtLinks()

                robot.straight(-400)
                robot.turn(90)
                bandeAusrichten()
                robot.straight(400)

                robot.straight(75)

            elif right == "black":
                robot.straight(50)
                robot.turn(92)

                if left == "blue":
                    vertauschtRechts()
                else:
                    vertauschtLinks()

                robot.straight(-350)
                robot.turn(90)
                bandeAusrichten()
                robot.straight(350)

                robot.straight(175)

            elif right == "green":
                robot.straight(125)
                robot.turn(92)

                if left == "black":
                    vertauschtRechts()
                else:
                    vertauschtLinks()
                
                robot.straight(-250)
                robot.turn(90)
                bandeAusrichten()
                robot.straight(250)

                robot.straight(200)

            if right == "red":
                robot.straight(205)
                robot.turn(92)

                if left == "green":
                    vertauschtRechts()
                else:
                    vertauschtLinks()
                
                robot.straight(-150)
                robot.turn(90)
                bandeAusrichten()
                robot.straight(150)
                
                robot.straight(275) 

            robot.turn(90)
            robot.straight(500)
            robot.turn(-90)

            pos[rightIndex] = "empty"
            pos[leftIndex] = "empty"

    # Prio 2 und 1
    else:
        if OutsiderIndex == 3:
            robot.straight(250)
        elif OutsiderIndex == 2:
            robot.straight(135)
        elif OutsiderIndex == 1:
            robot.straight(-15)
        elif OutsiderIndex == 0:
            robot.straight(-115)

        robot.turn(90)
        robot.stop()

        motor_left.run_angle(450, 175)
        motor_left.stop()
        robot.straight(100)
        grabber.run_time(-750, 2100)
        grabber.stop()
        robot.straight(-300)
        robot.stop()

        motor_left.run_angle(450, 225)
        motor_left.stop()
        robot.straight(-200)
        robot.stop()

        motor_left.run_angle(450, -385)
        motor_left.stop()
        robot.straight(-325)
        robot.turn(92)
        DriveTillDouble(9, 250)

        if whichIndexColor == "yellow":
            robot.straight(-325)
        elif whichIndexColor == "blue":
            robot.straight(-225)
        elif whichIndexColor == "black":
            robot.straight(15)
        elif whichIndexColor == "green":
            robot.straight(75)
        elif whichIndexColor == "red":
            robot.straight(175)

        robot.turn(90)

        robot.drive(250, 0)
        ev3.speaker.beep()
        wait(600)
        robot.stop()
        ev3.speaker.beep()
        wait(50)

        grabber.run_time(750, 2100)
        grabber.stop()
        robot.straight(-150)
        robot.turn(-89)

        #Zurückfahren
        if whichIndexColor == "yellow":    
            robot.straight(550)
        elif whichIndexColor == "blue":
            robot.straight(500)
        elif whichIndexColor == "black":
            robot.straight(425)
        elif whichIndexColor == "green":
            robot.straight(325)
        elif whichIndexColor == "red":
            robot.straight(250)

        robot.turn(-90)
        robot.drive(350, 0)
        ev3.speaker.beep()
        wait(2500)
        robot.stop()
        ev3.speaker.beep()
        wait(50)

        robot.straight(-145)
        robot.turn(-91)
        robot.straight(450)

        pos[OutsiderIndex] = "empty"

# ---- Main Programm ---- #
# Warten bis Mittelknopf gedrückt
ev3.screen.draw_text(x=10, y=10, text="Ready to start")
print("Programm is ready")
ev3.light.on(Color.ORANGE)
while Button.CENTER not in ev3.buttons.pressed():
    pass

pixy.set_lamp(1, 1)
ev3.screen.clear()
ev3.light.on(Color.GREEN)
ev3.speaker.beep()
wait(200)

def artefactsMain():
    global AllArtefacts 
    # Scannen
    scanDrive()

    while AllArtefacts > 0:
        print("----------------")
        print(pos)
        print("----------------")

        priority, whichIndex, whichIndexColor = checkPrio()
        print("Priority:", priority)
        print("Index of Red/Yellow:", whichIndex)

        collect_artefacts(priority, whichIndex, whichIndexColor)
        print("-----------------")
        print("One Run completed")
        print("-----------------")

        grabber.stop()
        robot.stop()

    DriveTillDouble(9, -350)

artefactsMain()