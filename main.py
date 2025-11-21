#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pixycamev3.pixy2 import Pixy2

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
motor_left = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
motor_right = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
# Negativ: Runter; Positiv: Hoch
front_grabber_bottom = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
# Negativ: Zu; Positiv: Auf
front_grabber_top = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
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
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=198.9)

# Einstellungen variable("robot")
robot.settings(600, 550, 450, 350)

#Fahren ohne DriveBase --> Etwas genauer anhalten, aber keine Distanz
def fahren(speeder):
    motor_left.run(speeder)
    motor_right.run(speeder)


def bremsen():
    motor_left.stop()
    motor_right.stop()


# Fahren bis zur einer doppelten schwarzen Linie
def DriveTillDouble(colorIndex, speed):
    colorReflection_left =  colorSensor_left.reflection()
    colorReflection_right = colorSensor_right.reflection()
    seenBlack = 0

    while seenBlack == 0: 
        colorReflection_left =  colorSensor_left.reflection()
        colorReflection_right = colorSensor_right.reflection()
        # Einfach gerade aus fahren mit bestimmter Geschwindigkeit (speed)
        fahren(speed)
        # Wenn beide Farbsensoren eine bestimmte Farbe (colorIndex) sehen, dann stoppen
        if colorReflection_left < colorIndex + 3 and colorReflection_left > colorIndex - 3 and colorReflection_right < colorIndex + 3 and colorReflection_right > colorIndex - 3:
            bremsen()
            ev3.speaker.beep()
            print("color sensor left: " + str(colorReflection_left))
            print("color sensor right: " + str(colorReflection_right))
            seenBlack = 1

def DriveTillColor(whichSensor, colorIndex, speed):
    if whichSensor == "left":
        colorReflection_left =  colorSensor_left.reflection()
        seenBlack = 0

        while seenBlack == 0: 
            colorReflection_left =  colorSensor_left.reflection()
            # Einfach gerade aus fahren mit variable speed
            fahren(speed)
            # Wenn ein ausgewählter (whichsensor) Farbsenor eine bestimmte Farbe (colorIndex) sieht, dann stoppen
            if colorReflection_left < colorIndex + 3 and colorReflection_left > colorIndex - 3:
                bremsen()
                ev3.speaker.beep()
                print("color sensor left: " + str(colorReflection_left))
                seenBlack = 1

    elif whichSensor == "right":
        colorReflection_right =  colorSensor_right.reflection()
        seenBlack = 0

        while seenBlack == 0: 
            colorReflection_right =  colorSensor_right.reflection()
            # Einfach gerade aus fahren mit variable speed
            fahren(speed)
            # Wenn ein ausgewählter (whichsensor) Farbsenor eine bestimmte Farbe (colorIndex) sieht, dann stoppen
            if colorReflection_right < colorIndex + 3 and colorReflection_right > colorIndex - 3:
                bremsen()
                ev3.speaker.beep()
                print("color sensor left: " + str(colorReflection_right))
                seenBlack = 1


#Linienfolger mit Bedingung --> Bis zu einer doppelten schwarzen Linie
def LineFollower_tillDouble():
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

#Erkennen der Süßigkeitenkiste
def KisteErkennen():
    minimal_middle = 130
    maximal_middle = 155

    while True:
        nr_blocks, blocks = pixy.get_blocks(1, 1)
        pixy.set_lamp(0, 0)
        # Extract data of first (and only) block
        if nr_blocks >= 1:
            sig = blocks[0].sig
            x_kiste = blocks[0].x_center

            # Wenn nicht Mitte (Wert x_kiste 1 bis x_kiste), dann drehen
            if x_kiste <= minimal_middle or x_kiste >= maximal_middle:
                if x_kiste < minimal_middle:
                    robot.drive(0, -50)
                    lastDirection = "left"
                    #print(x_kiste)
                    #print("left")
                elif x_kiste > maximal_middle:
                    robot.drive(0, 50)
                    lastDirection = "right"
                    #print(x_kiste)
                    #print("right")

            else:
                robot.stop()
                print("Stehe Mitte")
                #print(x_kiste)
                return True
            
def checkUp():
    robot.straight(-100)
    front_grabber_top.run_angle(-300, 50)
    wait(100)
    front_grabber_top.run_angle(-300, -50)
    robot.turn(90)
    wait(250)
    robot.turn(90)
    wait(250)
    robot.turn(90)
    wait(250)
    robot.turn(90)
    wait(250)
    robot.straight(90)
    wait(10000)

##---------------------------- Fahrprogramm ----------------------------##
## Anfahren ##
#checkUp()
DriveTillColor("left", 7, 400)
bremsen()
robot.straight(25)
wait(100)
robot.turn(-90)
robot.stop()
LineFollower_tillDouble()
bremsen()
robot.straight(75)
wait(200)
robot.turn(90)
robot.straight(575)
robot.stop()
DriveTillColor("right", 7, 400)
LineFollower_tillDouble()
bremsen()
robot.straight(300)
robot.turn(180)

## In die Kiste grabben ##
isMiddle = KisteErkennen()

if isMiddle == True:
    robot.stop()
    front_grabber_top.run_angle(-300, 50)
    print("Gehe runter")

    robot.straight(-175)
    robot.straight(35)
    front_grabber_bottom.run_time(-400, 1300)
    front_grabber_bottom.run_angle(200, 5, then=Stop.HOLD)
    front_grabber_top.run_time(400, 550)
    front_grabber_bottom.run_time(250, 1500, then=Stop.COAST)