import time
import board
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
Skit = ServoKit(channels=16)

Servopin = 7
centered = 70
max_left = 30
max_right = 110

def center():
    Skit.servo[Servopin].angle = centered

def right(Angle):
    P1 = centered + Angle
    if P1 > 110:
        P1 = 110
    Skit.servo[Servopin].angle = P1
    
def left(Angle):
    P2 = centered - Angle
    if P2 < 30:
        P2 = 30
    Skit.servo[Servopin].angle = P2