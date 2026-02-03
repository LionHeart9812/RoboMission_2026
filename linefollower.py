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

seenBlack = 0
i = 1
lastDirection = "none"

#Gestell des Roboters (Durchmessser der Reifen, Radstand)
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=199)

# Einstellungen variable("robot")
robot.settings(600, 550, 200, 150)

def fahren(speeder):
    robot.drive(speeder, 0)

def bremsen():
    robot.stop()

# Fahren bis zur einer doppelten schwarzen Linie
def DriveTillDouble(colorIndex, speed):
    colorReflection_left =  colorSensor_left.reflection()
    colorReflection_right = colorSensor_right.reflection()
    seenBlack = 0
    fahren(speed)

    while seenBlack == 0: 
        colorReflection_left =  colorSensor_left.reflection()
        colorReflection_right = colorSensor_right.reflection()
        # Einfach gerade aus fahren mit bestimmter Geschwindigkeit (speed)
        # Wenn beide Farbsensoren eine bestimmte Farbe (colorIndex) sehen, dann stoppen
        if colorReflection_left < colorIndex + 3 and colorReflection_left > colorIndex - 3 and colorReflection_right < colorIndex + 3 and colorReflection_right > colorIndex - 3:
            bremsen()
            ev3.speaker.beep()
            print("color sensor left: " + str(colorReflection_left))
            print("color sensor right: " + str(colorReflection_right))
            print("----------------------------")
            seenBlack = 1

def DriveTillColor(whichSensor, colorIndex, speed):
    if whichSensor == "left":
        colorReflection_left =  colorSensor_left.reflection()
        seenBlack = 0
        fahren(speed)

        while seenBlack == 0: 
            colorReflection_left =  colorSensor_left.reflection()
            # Einfach gerade aus fahren mit variable speed
            # Wenn ein ausgewählter (whichsensor) Farbsenor eine bestimmte Farbe (colorIndex) sieht, dann stoppen
            if colorReflection_left < colorIndex + 3 and colorReflection_left > colorIndex - 3:
                bremsen()
                ev3.speaker.beep()
                print("color sensor left: " + str(colorReflection_left))
                print("----------------------------")
                seenBlack = 1

    elif whichSensor == "right":
        colorReflection_right =  colorSensor_right.reflection()
        seenBlack = 0
        fahren(speed)

        while seenBlack == 0: 
            colorReflection_right =  colorSensor_right.reflection()
            # Einfach gerade aus fahren mit variable speed
            # Wenn ein ausgewählter (whichsensor) Farbsenor eine bestimmte Farbe (colorIndex) sieht, dann stoppen
            if colorReflection_right < colorIndex + 3 and colorReflection_right > colorIndex - 3:
                bremsen()
                ev3.speaker.beep()
                print("color sensor left: " + str(colorReflection_right))
                print("----------------------------")
                seenBlack = 1


#Linienfolger mit Bedingung --> Bis zu einer doppelten schwarzen Linie
def LF_tillDoubleBlack():
    colorReflection_left =  colorSensor_left.reflection()
    colorReflection_right = colorSensor_right.reflection()
    seenBlack = 0
    seeingNothing = False

    while seenBlack == 0:
        colorReflection_left =  colorSensor_left.reflection()
        colorReflection_right = colorSensor_right.reflection()

        if colorReflection_left < 3 and colorReflection_right < 3 and seeingNothing == False:
            seeingNothing = True

        # Wenn linker Farbsensor auf schwarz, fahre etwas nach rechts
        elif colorReflection_left < 15 and seeingNothing == False:
            print("Links")
            robot.stop()
            while colorReflection_left < 15 and seenBlack == 0:
                colorReflection_left =  colorSensor_left.reflection()
                colorReflection_right = colorSensor_right.reflection()

                robot.drive(100, -40)
                seenBlack = 0

                if colorReflection_left < 3 and colorReflection_right < 3:
                    seeingNothing = True

                # Wenn beide schwarz stoppen
                if colorReflection_right < 15 and colorReflection_left < 15 and seeingNothing == False:
                    seenBlack = 1
                    robot.stop()
                    bremsen()
                    ev3.speaker.beep()
                    print("color sensor left: " + str(colorReflection_left))
                    print("color sensor right: " + str(colorReflection_right))
                    print("----------------------------")

        # Wenn rechter, dann etwas nach links
        elif colorReflection_right < 15 and seeingNothing == False:
            print("Rechts")
            robot.stop()
            while colorReflection_right < 15 and seenBlack == 0:
                colorReflection_left =  colorSensor_left.reflection()
                colorReflection_right = colorSensor_right.reflection()

                robot.drive(100, 40)
                seenBlack = 0

                if colorReflection_left < 3 and colorReflection_right < 3:
                    seeingNothing = True

                # Wenn beide schwarz stoppen
                if colorReflection_right < 15 and colorReflection_left < 15 and seeingNothing == False:
                    seenBlack = 1
                    robot.stop()
                    bremsen()
                    ev3.speaker.beep()
                    print("color sensor left: " + str(colorReflection_left))
                    print("color sensor right: " + str(colorReflection_right))
                    print("----------------------------")

        # Wenn kein Sensor auf schwarz, oder nichts erkennen, fahre geradeaus
        elif colorReflection_right > 14 and colorReflection_left > 14 or seeingNothing == True:
            robot.stop()
            print("Geradeaus")
            while colorReflection_right > 14 and colorReflection_left > 14 or seeingNothing == True:
                colorReflection_left =  colorSensor_left.reflection()
                colorReflection_right = colorSensor_right.reflection()

                robot.drive(300, 0)
                seenBlack = 0

                if colorReflection_left > 2 or colorReflection_right > 2:
                    seeingNothing = False

#Linienfolger mit Bedingung --> Bis Zeit vrbei ist
def LF_tillTime(timer):
    colorReflection_left = colorSensor_left.reflection()
    colorReflection_right = colorSensor_right.reflection()
    seenBlack = 0
    seeingNothing = False
    start = time.time()

    # läuft, bis entweder Schwarz gefunden wurde oder die Zeit abgelaufen ist
    while seenBlack == 0 and time.time() - start <= timer:
        colorReflection_left = colorSensor_left.reflection()
        colorReflection_right = colorSensor_right.reflection()

        # Beide Sensoren sehen sehr dunkel -> nichts / Unterbrechung
        if colorReflection_left < 3 and colorReflection_right < 3 and not seeingNothing:
            seeingNothing = True

        # Linker Sensor auf Linie -> nach rechts korrigieren
        elif colorReflection_left < 15 and not seeingNothing:
            print("Links")
            robot.stop()
            while (colorReflection_left < 15 and seenBlack == 0 
                   and time.time() - start <= timer):
                colorReflection_left = colorSensor_left.reflection()
                colorReflection_right = colorSensor_right.reflection()

                robot.drive(100, -40)

                if colorReflection_left < 3 and colorReflection_right < 3:
                    seeingNothing = True

                # Beide schwarz ODER Zeit vorbei -> stoppen
                if ((colorReflection_right < 15 and colorReflection_left < 15 
                     and not seeingNothing)
                        or time.time() - start > timer):
                    seenBlack = 1
                    robot.stop()
                    bremsen()
                    ev3.speaker.beep()
                    print("color sensor left: " + str(colorReflection_left))
                    print("color sensor right: " + str(colorReflection_right))
                    print("----------------------------")

        # Rechter Sensor auf Linie -> nach links korrigieren
        elif colorReflection_right < 15 and not seeingNothing:
            print("Rechts")
            robot.stop()
            while (colorReflection_right < 15 and seenBlack == 0 
                   and time.time() - start <= timer):
                colorReflection_left = colorSensor_left.reflection()
                colorReflection_right = colorSensor_right.reflection()

                robot.drive(100, 40)

                if colorReflection_left < 3 and colorReflection_right < 3:
                    seeingNothing = True

                # Beide schwarz ODER Zeit vorbei -> stoppen
                if ((colorReflection_right < 15 and colorReflection_left < 15 
                     and not seeingNothing)
                        or time.time() - start > timer):
                    seenBlack = 1
                    robot.stop()
                    bremsen()
                    ev3.speaker.beep()
                    print("color sensor left: " + str(colorReflection_left))
                    print("color sensor right: " + str(colorReflection_right))
                    print("----------------------------")

        # Kein Sensor auf Linie ODER „nichts gesehen“ -> geradeaus suchen
        elif (colorReflection_right > 14 and colorReflection_left > 14) or seeingNothing:
            robot.stop()
            print("Geradeaus")
            while (((colorReflection_right > 14 and colorReflection_left > 14) 
                    or seeingNothing)
                   and seenBlack == 0
                   and time.time() - start <= timer):

                colorReflection_left = colorSensor_left.reflection()
                colorReflection_right = colorSensor_right.reflection()

                robot.drive(300, 0)

                # Sobald wieder vernünftige Werte da sind, seeingNothing zurücksetzen
                if colorReflection_left > 2 or colorReflection_right > 2:
                    seeingNothing = False

            robot.stop()

    # Sicherheit: nach Ablauf der Zeit stoppen
    robot.stop()

def LF_tillDoubleNEW(colorIndex, speed):
    colorReflection_left =  colorSensor_left.reflection()
    colorReflection_right = colorSensor_right.reflection()
    seenBlack = 0
    

    while seenBlack == 0: 
        colorReflection_left =  colorSensor_left.reflection()
        colorReflection_right = colorSensor_right.reflection()
        
        # Linienfolger
        correctionLeft = round((colorReflection_left + speed) * 0.5)
        correctionRight = round((colorReflection_right + speed) * 0.5)

        motor_left.dc(correctionLeft)
        motor_right.dc(correctionRight)
        print("Linkes:", correctionLeft)
        print("Rechtes:", correctionRight)

        # Wenn beide Farbsensoren eine bestimmte Farbe (colorIndex) sehen, dann stoppen
        if colorReflection_left < colorIndex + 3 and colorReflection_left > colorIndex - 3 and colorReflection_right < colorIndex + 3 and colorReflection_right > colorIndex - 3:
            motor_left.stop()
            motor_right.stop()
            ev3.speaker.beep()
            print("color sensor left: " + str(colorReflection_left))
            print("color sensor right: " + str(colorReflection_right))
            print("----------------------------")
            seenBlack = 1