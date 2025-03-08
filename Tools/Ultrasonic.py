# Bibliotheken einbinden
import RPi.GPIO as GPIO
import time

# GPIO Modus (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# GPIO Pins zuweisen GPIO:25,23,24
GPIO_TRIGGER_R = 24
GPIO_ECHO_R = 24

GPIO_TRIGGER_L = 23
GPIO_ECHO_L = 23

GPIO_TRIGGER_V = 25
GPIO_ECHO_V = 25

last_distL = 0.0
last_distR = 0.0
last_distV = 0.0

# Richtung der GPIO-Pins festlegen (IN / OUT)
GPIO.setup(GPIO_TRIGGER_R, GPIO.OUT)
GPIO.setup(GPIO_ECHO_R, GPIO.IN)

GPIO.setup(GPIO_TRIGGER_L, GPIO.OUT)
GPIO.setup(GPIO_ECHO_L, GPIO.IN)

GPIO.setup(GPIO_TRIGGER_V, GPIO.OUT)
GPIO.setup(GPIO_ECHO_V, GPIO.IN)


def distanz_R(timeout=0.02):
    global last_distR
    mytimeout = timeout
    GPIO.setup(GPIO_TRIGGER_R, GPIO.OUT)
    GPIO.output(GPIO_TRIGGER_R, True)

    time.sleep(0.0001)
    GPIO.output(GPIO_TRIGGER_R, False)
    GPIO.setup(GPIO_ECHO_R, GPIO.IN)
    
    distanz_R = 0.0


    StartZeit = time.time()
    StopZeit = time.time()
    mytime = time.time()

    # speichere Startzeit
    while GPIO.input(GPIO_ECHO_R) == 0 and (time.time() - mytime) < mytimeout:
        StartZeit = time.time()

    # speichere Ankunftszeit
    while GPIO.input(GPIO_ECHO_R) == 1 and (time.time() - StartZeit) < mytimeout:
        StopZeit = time.time()

    # Zeit Differenz zwischen Start und Ankunft
    TimeElapsed = StopZeit - StartZeit
    distanz_R = (TimeElapsed * 34300) / 2
    
    if distanz_R > 2.0 and distanz_R < 300.0:
        last_distR = distanz_R 
        
    else:
        distanz_R = last_distR

    return distanz_R


def distanz_L(timeout=0.02):
    global last_distL
    mytimeout = timeout
    GPIO.setup(GPIO_TRIGGER_L, GPIO.OUT)
    GPIO.output(GPIO_TRIGGER_L, True)

    time.sleep(0.0001)
    GPIO.output(GPIO_TRIGGER_L, False)
    GPIO.setup(GPIO_ECHO_L, GPIO.IN)
    
    distanz_L = 0.0

    StartZeit = time.time()
    StopZeit = time.time()
    mytime = time.time()

    # speichere Startzeit
    while GPIO.input(GPIO_ECHO_L) == 0 and (time.time() - mytime) < mytimeout:
        StartZeit = time.time()

    # speichere Ankunftszeit
    while GPIO.input(GPIO_ECHO_L) == 1 and (time.time() - StartZeit) < mytimeout:
        StopZeit = time.time()

    # Zeit Differenz zwischen Start und Ankunft
    TimeElapsed = StopZeit - StartZeit
    distanz_L = (TimeElapsed * 34300) / 2
    
    if distanz_L > 2.0 and distanz_L < 300.0:
            last_distL = distanz_L
            
    else:
        distanz_L = last_distL

    return distanz_L


def distanz_V(timeout=0.02):
    global last_distV
    mytimeout = timeout
    GPIO.setup(GPIO_TRIGGER_V, GPIO.OUT)
    GPIO.output(GPIO_TRIGGER_V, True)

    time.sleep(0.0001)
    GPIO.output(GPIO_TRIGGER_V, False)
    GPIO.setup(GPIO_ECHO_V, GPIO.IN)

    distanz_V = 0.0

    StartZeit = time.time()
    StopZeit = time.time()
    mytime = time.time()

    # speichere Startzeit
    while GPIO.input(GPIO_ECHO_V) == 0 and (time.time() - mytime) < mytimeout:
        StartZeit = time.time()

    # speichere Ankunftszeit
    while GPIO.input(GPIO_ECHO_V) == 1 and (time.time() - StartZeit) < mytimeout:
        StopZeit = time.time()

    # Zeit Differenz zwischen Start und Ankunft
    TimeElapsed = StopZeit - StartZeit
    distanz_V = (TimeElapsed * 34300) / 2
    
    if distanz_V > 2.0 and distanz_V < 300.0:
            last_distV = distanz_V 
        
    else:
        distanz_V = last_distV


    return distanz_V 


if __name__ == '__main__':
    try:
        while True:
            abstand_L = distanz_L()
            print("Entfernung links = %.1f cm" % abstand_L)
            abstand_R = distanz_R()
            print("                             Entfernung rechts = %.1f cm" % abstand_R)
            abstand_V = distanz_V()
            print("                                                                  Entfernung vorne = %.1f cm" % abstand_V)
            time.sleep(0.2)

    # Beim Abbruch durch STRG+C resetten
    except KeyboardInterrupt:
        print("Messung vom User gestoppt")
        GPIO.cleanup()

