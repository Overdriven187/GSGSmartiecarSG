import time
#import board
import RPi.GPIO as GPIO

#Richtungspin: GPIO 26
#Speedpin: GPIO 13
GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
pwm = GPIO.PWM(13,650)

speed = 20
print("Starte Motor vorwärts")
#motor auf pin 8
GPIO.output(26, GPIO.HIGH)
pwm.start(speed)
time.sleep(3.0)

#Stop:
print("Stop")
pwm.stop()
time.sleep(3.0)

print("Rückwärts")
GPIO.output(26, GPIO.LOW)
pwm.start(speed)
time.sleep(3.0)

#Stop:
print("Stop")
pwm.stop()
time.sleep(3.0)

GPIO.cleanup()
