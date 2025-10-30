#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
motor_left = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
motor_right = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
# Negativ: Runter; Positiv: Hoch
front_grabber = Motor(Port.C, positive_direction=Direction.CLOCKWISE)
# Negativ: Runter; Positiv: Hoch
big_grabber = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
colorSensor_left = ColorSensor(Port.S1)
colorSensor_right = ColorSensor(Port.S4)
colorReflection_left =  colorSensor_left.reflection()
colorReflection_right = colorSensor_right.reflection()
seenBlack = 0
i = 1

# Write your program here.
ev3.speaker.beep()

#Gestell des Roboters (Durchmessser der Reifen, Radstand)
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=196.5)


# Einstellungen variable("robot")
robot.settings(600, 550, 450, 450)

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

                robot.drive(375, 0)
                seenBlack = 0

                if colorReflection_left > 2 or colorReflection_right > 2:
                    seeingNothing = False

##---------------------------- Fahrprogramm ----------------------------##
#-------------- Anfahren --------------#
robot.straight(50)
robot.stop()           

DriveTillColor("left", 10, 300)
bremsen()

robot.straight(60)
robot.stop()

robot.turn(-90)
robot.stop()

DriveTillDouble(5, 300)
bremsen()

#-------------- Aufgabe MarsRover --------------#
# Marsrover umklappen
robot.straight(-25)
robot.turn(90)
robot.straight(120)
front_grabber.run_angle(250, -85)
robot.straight(-150)
wait(100)
front_grabber.stop()
front_grabber.run_angle(500, 85)
robot.turn(-90)
robot.straight(120)
robot.turn(-90)

#-------------- Aufgabe Balls --------------#
# #Roboter fährt in Ballkäfig
robot.straight(79)
front_grabber.run_angle(600, -101)
robot.straight(60)
robot.straight(-55)
robot.straight(55)

robot.straight(-100)
robot.stop()

front_grabber.run_angle(200, 45)

#Roboter dreht und fährt 
robot.turn(90)
robot.stop()
DriveTillDouble(5, 300)
bremsen()
robot.turn(-90)
robot.straight(150)

# front_grabber.run_angle(150, 40)
# wait(100)
# front_grabber.run_angle(150, -40)
# wait(300)
# front_grabber.run_angle(150, 40)
# wait(100)
# front_grabber.run_angle(150, -40)
# front_grabber.stop()

robot.stop()
bremsen()