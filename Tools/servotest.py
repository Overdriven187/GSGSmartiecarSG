import time
import board
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
Skit = ServoKit(channels=16)

mitte = 70

# Servo auf mitte
Skit.servo[7].angle = mitte
print("Servo Mitte")
time.sleep(5.0)

while (True):

    #Servo nach links
    Skit.servo[7].angle = 30
    print("Servo links")
    time.sleep(2.0)

    # Servo auf mitte
    Skit.servo[7].angle = mitte
    print("Servo Mitte")
    time.sleep(2.0)


    #Servo nach rechts
    Skit.servo[7].angle = 110
    print("Servo rechts")
    time.sleep(2.0)

    # Servo auf mitte
    Skit.servo[7].angle = mitte
    print("Servo Mitte")
    time.sleep(2.0)


