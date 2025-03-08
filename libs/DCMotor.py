import RPI.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Global PWM object for GSM1
pwmMotor = None

# DC Motor 1
GSM1 = 5
in1 = 6
in2 = 7

# backward
def runMotor_R(Speed):
    tmpSpeed = 0
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    if Speed < 185:
        tmpSpeed = 185
        # analogWrite(GSM1, Speed);
        pwmMotor.ChangeDutyCycle((Speed / 255.0) * 100)
        time.sleep(0.1)  # delay(100);
    # analogWrite(GSM1, Speed);
    pwmMotor.ChangeDutyCycle((Speed / 255.0) * 100)

# stop
def stopMotor():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    # analogWrite(GSM1, 0);
    pwmMotor.ChangeDutyCycle(0)
    # active breaking then stop
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    # analogWrite(GSM1, 185);
    pwmMotor.ChangeDutyCycle((185 / 255.0) * 100)
    time.sleep(0.05)  # delay(50);
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    # analogWrite(GSM1, 0);
    pwmMotor.ChangeDutyCycle(0)

# run motor
def runMotor(Speed):
    tmpSpeed = 0
    GPIO.output(in1, GPIO.HIGH)  # motor 1 begins to rotate forward
    GPIO.output(in2, GPIO.LOW)
    if Speed < 185:
        tmpSpeed = 185
        # analogWrite(GSM1, Speed); // motor 1 should drive forward with the maximum speed
        pwmMotor.ChangeDutyCycle((Speed / 255.0) * 100)
        time.sleep(0.1)  # delay(100);
    # analogWrite(GSM1, Speed);
    pwmMotor.ChangeDutyCycle((Speed / 255.0) * 100)