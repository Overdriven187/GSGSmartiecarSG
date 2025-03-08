import time
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit

Skit = ServoKit(channels=16)
r = 10
g = 11
b = 9

# =====================================================


def led_r():
    Skit.servo[r].angle = 100
    Skit.servo[g].angle = None 
    Skit.servo[b].angle = None 

def led_g():
    Skit.servo[r].angle = None 
    Skit.servo[g].angle = 10 
    Skit.servo[b].angle = None 

def led_b():
    Skit.servo[r].angle = None 
    Skit.servo[g].angle = None 
    Skit.servo[b].angle = 10 

def led_y():
    Skit.servo[r].angle = 100 
    Skit.servo[g].angle = 40 
    Skit.servo[b].angle = None 

def led_off():
    Skit.servo[r].angle = None 
    Skit.servo[g].angle = None 
    Skit.servo[b].angle = None 
    
#=====================================================    

if __name__ == '__main__':
    try:
        led_r()
        time.sleep(2)
        led_g()
        time.sleep(2)
        led_b()
        time.sleep(2)
        led_y()
    except KeyboardInterrupt:
        print("Messung vom User gestoppt")
        GPIO.cleanup()