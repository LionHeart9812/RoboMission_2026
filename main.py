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
robot = DriveBase(motor_left, motor_right, wheel_diameter=60, axle_track=200)

# Einstellungen variable("robot")
robot.settings(600, 550, 200, 150)

#Fahren ohne DriveBase --> Etwas genauer anhalten, aber keine Distanz
def fahren(speeder):
    motor_left.run(speeder)
    motor_right.run(speeder)


def bremsen():
    motor_left.stop()
    motor_right.stop()

#Erkennen der Süßigkeitenkiste
def KisteErkennen():
    minimal_middle = 147.5
    maximal_middle = 162.5

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
                    robot.drive(0, -40)
                    lastDirection = "left"
                    #print(x_kiste)
                    #print("left")
                elif x_kiste > maximal_middle:
                    robot.drive(0, 40)
                    lastDirection = "right"
                    #print(x_kiste)
                    #print("right")

            else:
                robot.stop()
                print("Stehe Mitte")
                print("X-Koordinate von der Kiste:", x_kiste)
                return True
            
def BonbonErkennen():
    start = time.time()

    while time.time() - start < 2.75:
        try:
            # 6 = Signature 1 + 2 + 3
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

            if colorSIG == 2:
                return "purple"
            elif colorSIG == 3:
                return "green"
            elif colorSIG == 1:
                return "orange"
            
        time.sleep(0.2)

    # Wenn in der Zeitspanne nichts Passendes erkannt wurde
    return None
            

def driveToPlace(whichColor):
    driveDistance = -500
    while True:
        # Lila, rechte Seite + rechts
        if whichColor == "purple":
            robot.turn(-10)
            robot.stop()
            fahren(500)
            time.sleep(3.5)
            robot.stop()
            robot.straight(-150)
            robot.turn(90)
            robot.straight(-100)
            robot.stop()
            DriveTillDouble(7, -500)
            bremsen()
            motor_left.run_angle(-500, -300)
            robot.straight(55)
        
            front_grabber_bottom.run_angle(125, -225)
            front_grabber_top.run_angle(-125, 50)
            time.sleep(0.5)
            front_grabber_top.run_angle(125, 50)
            front_grabber_bottom.run_angle(125, 225)
            robot.turn(-60)
            robot.stop()
            fahren(500)
            time.sleep(1.5)
            bremsen()
            robot.straight(driveDistance)
            robot.turn(-90)
            robot.straight(driveDistance)
            robot.turn(-90)
            robot.straight(driveDistance)
            robot.stop()
            return
        
        # Grün , linke Seite + rechts
        elif whichColor == "green":
            robot.turn(190)
            robot.stop()
            fahren(500)
            time.sleep(3.5)
            robot.stop()
            robot.straight(-150)
            robot.turn(-90)
            robot.straight(-100)
            robot.stop()
            DriveTillDouble(7, -500)
            bremsen()
            motor_left.run_angle(-500, -300)
            robot.straight(55)

            front_grabber_bottom.run_angle(125, -225)
            front_grabber_top.run_angle(-125, 50)
            time.sleep(0.5)
            front_grabber_top.run_angle(125, 50)
            front_grabber_bottom.run_angle(125, 225)
            robot.turn(-130)
            robot.stop()
            fahren(500)
            time.sleep(1.5)
            bremsen()
            robot.turn(90)
            robot.straight(driveDistance)
            robot.turn(90)
            robot.straight(driveDistance)
            robot.turn(90)
            robot.stop()
            return
        
        # Orange, rechts Seite + rechts
        elif whichColor == "orange":
            robot.turn(-10)
            robot.stop()
            fahren(500)
            time.sleep(3.5)
            robot.stop()
            robot.straight(-150)
            robot.turn(90)
            robot.straight(-100)
            robot.stop()
            DriveTillDouble(7, -500)
            bremsen()
            motor_right.run_angle(-500, -300)
            robot.straight(35)

            front_grabber_bottom.run_angle(125, -225)
            front_grabber_top.run_angle(-125, 50)
            time.sleep(0.5)
            front_grabber_top.run_angle(125, 50)
            front_grabber_bottom.run_angle(125, 225)
            robot.turn(-40)
            robot.stop()
            fahren(500)
            time.sleep(1.5)
            bremsen()
            robot.straight(driveDistance)
            robot.turn(-90)
            robot.straight(driveDistance)
            robot.turn(-90)
            robot.straight(driveDistance)
            robot.stop()
            return
        
        # Nichts
        else:
            while True:
                print("Andere male")
                robot.straight(-10)
                robot.turn(90)

                isMiddle = KisteErkennen()

                if isMiddle == True:
                    robot.stop()
                    front_grabber_top.run_angle(-300, 65)

                    robot.straight(-185)
                    robot.straight(25)
                    front_grabber_bottom.run_time(-550, 1400)
                    #front_grabber_bottom.run_angle(200, 5, then=Stop.HOLD)
                    front_grabber_top.run_time(550, 525)
                    front_grabber_bottom.run_time(125, 1800, then=Stop.COAST)

                    ## Tütchen analysieren ##
                    robot.straight(200)
                    robot.turn(-90)
                    front_grabber_bottom.run_angle(125, -200)

                    detectedColor2 = BonbonErkennen()
                    print("Erkannte Farbe:", detectedColor2)
                    ev3.speaker.beep()
                    front_grabber_bottom.run_time(125, 1800, then=Stop.COAST)
                    front_grabber_bottom.stop()
                    robot.stop()
                    if detectedColor2 != None:
                        driveToPlace(detectedColor2)
            
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
    time.sleep(100)

##---------------------------- Fahrprogramm ----------------------------##
## Anfahren ##
#checkUp()
robot.straight(-200)
robot.stop()
DriveTillColor("right", 7, -300)
bremsen()
wait(50)
robot.turn(90)
robot.stop()
DriveTillDouble(7, 400)
bremsen()
robot.straight(80)
wait(100)
robot.turn(91)
robot.straight(575)
robot.stop()
DriveTillColor("right", 7, 400)
bremsen()
wait(100)
robot.straight(50)
robot.stop()
wait(100)
LineFollower_tillDouble()
bremsen()
wait(100)
robot.straight(300)
robot.turn(180)

while True:
    ## In die Kiste grabben ##
    print("Erstes mal erkennen")
    isMiddle = KisteErkennen()

    if isMiddle == True:
        robot.stop()
        front_grabber_top.run_angle(-300, 65)

        robot.straight(-200)
        robot.straight(25)
        front_grabber_bottom.run_time(-550, 1400)
        front_grabber_top.run_time(550, 525)
        front_grabber_bottom.run_time(125, 1800, then=Stop.COAST)

        ## Tütchen analysieren ##
        robot.straight(200)
        robot.turn(-90)
        front_grabber_bottom.run_angle(125, -200)

        detectedColor = BonbonErkennen()
        print("Erkannte Farbe:", detectedColor)
        ev3.speaker.beep()
        front_grabber_bottom.run_time(125, 1800, then=Stop.COAST)
        front_grabber_bottom.stop()
        robot.stop()

        if detectedColor is not None:
            driveToPlace(detectedColor)
        else:
            print("Keine Farbe erkannt, neue Runde.")
            robot.turn(90)
            robot.straight(-40)
            robot.stop()
