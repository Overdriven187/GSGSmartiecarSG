import time
import RPI.GPIO as GPIO  # Assuming Raspberry Pi or similar GPIO library

# Define pin numbers for GPIO
trigechoPin_F = 1  # A1 equivalent
trigechoPin_L = 2  # A2 equivalent  
trigechoPin_R = 0  # A0 equivalent

# Global variables
distance = 0
pulseWidthUs = 0
dist_Left = 0
dist_Right = 0
dist_Front = 0

# The ultrasonic velocity (cm/us) compensated by temperature
def VELOCITY_TEMP(temp):
    return ((331.5 + 0.6 * float(temp)) * 100 / 1000000.0)

# Helper functions to simulate Arduino functionality
def pinMode(pin, mode):
    if mode == GPIO.OUT:
        GPIO.setup(pin, GPIO.OUT)
    else:
        GPIO.setup(pin, GPIO.IN)

def digitalWrite(pin, value):
    GPIO.output(pin, value)

def delayMicroseconds(us):
    time.sleep(us / 1000000.0)

def delay(ms):
    time.sleep(ms / 1000.0)

def pulseIn(pin, state):
    # Wait for the pin to change to the specified state
    start = time.time() * 1000000  # Convert to microseconds
    
    # Wait for pin to go to desired state
    while GPIO.input(pin) != state:
        if (time.time() * 1000000 - start) > 1000000:  # 1 second timeout
            return 0
    
    # Measure time in desired state
    start = time.time() * 1000000
    while GPIO.input(pin) == state:
        if (time.time() * 1000000 - start) > 1000000:  # 1 second timeout
            return 0
    
    return time.time() * 1000000 - start  # Return duration in microseconds

def SpaceUltraSonicFront():
    global dist_Front, pulseWidthUs
    # Sending the trigger pulse
    dist = 0

    pinMode(trigechoPin_F, GPIO.OUT)
    digitalWrite(trigechoPin_F, GPIO.LOW)
    delayMicroseconds(2)
    digitalWrite(trigechoPin_F, GPIO.HIGH)
    delayMicroseconds(10)
    digitalWrite(trigechoPin_F, GPIO.LOW)
    
    # Wait for echo pin to go high
    pinMode(trigechoPin_F, GPIO.IN)
    pulseWidthUs = pulseIn(trigechoPin_F, GPIO.HIGH)
    dist = pulseWidthUs * VELOCITY_TEMP(20) / 2.0

    if (dist < 2) or (dist > 350):
        dist = dist_Front
    else:
        dist_Front = dist
        
    delay(20)
    # Returns integer in cm
    return int(dist)

def SpaceUltraSonicLeft():
    global dist_Left, pulseWidthUs
    # Sending the trigger pulse
    dist = 0

    pinMode(trigechoPin_L, GPIO.OUT)
    digitalWrite(trigechoPin_L, GPIO.LOW)
    delayMicroseconds(2)
    digitalWrite(trigechoPin_L, GPIO.HIGH)
    delayMicroseconds(10)
    digitalWrite(trigechoPin_L, GPIO.LOW)
    
    # Wait for echo pin to go high
    pinMode(trigechoPin_L, GPIO.IN)
    pulseWidthUs = pulseIn(trigechoPin_L, GPIO.HIGH)
    dist = pulseWidthUs * VELOCITY_TEMP(20) / 2.0

    if (dist < 2) or (dist > 350):
        dist = dist_Left
    else:
        dist_Left = dist
        
    delay(20)
    # Returns integer in cm
    return int(dist)

def SpaceUltraSonicRight():
    global dist_Right, pulseWidthUs
    # Sending the trigger pulse
    dist = 0

    pinMode(trigechoPin_R, GPIO.OUT)
    digitalWrite(trigechoPin_R, GPIO.LOW)
    delayMicroseconds(2)
    digitalWrite(trigechoPin_R, GPIO.HIGH)
    delayMicroseconds(10)
    digitalWrite(trigechoPin_R, GPIO.LOW)
    
    # Wait for echo pin to go high
    pinMode(trigechoPin_R, GPIO.IN)
    pulseWidthUs = pulseIn(trigechoPin_R, GPIO.HIGH)
    dist = pulseWidthUs * VELOCITY_TEMP(20) / 2.0

    if (dist < 2) or (dist > 350):
        dist = dist_Right
    else:
        dist_Right = dist
        
    delay(20)
    # Returns integer in cm
    return int(dist)

# Empty function for sensor type compatibility
def ultrasonicstart():
    return

# Initialize GPIO at import time
def initialize():
    GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
    
    # Setup pins
    GPIO.setup(trigechoPin_F, GPIO.OUT)
    GPIO.setup(trigechoPin_L, GPIO.OUT)
    GPIO.setup(trigechoPin_R, GPIO.OUT)

# Call initialize function
initialize()