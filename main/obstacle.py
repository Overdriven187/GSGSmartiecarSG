import time
import RPI.GPIO as GPIO
from Tools import Led as L
from libs import DCMotor as DC
from libs import steering as ST
from libs import gyro as GY
from libs import ultrasonic as US
from libs import cam as CAM  # Assuming a cam module exists for camera functions

# Global variables and simulation parameters
prog_start_time = time.time()

# Button configuration
Button = 4  # Same as in original Arduino code
LOW = 0
HIGH = 1

# Arduino-like functions
def delay(ms):
    time.sleep(ms / 1000.0)

def millis():
    return int((time.time() - prog_start_time) * 1000)

def digitalRead(pin):
    return GPIO.input(pin)

# Global variables
start_time = 0
corners = 0
Distance_Front = 0
Distance_Left = 0
Distance_Right = 0
angle = 0.0
danger = 0.0
correction_Left = 10.0
correction_Right = 10.0
StraightAngle = 0.0
DrivingDirection = 'U'  # 'U' for Unknown
NormalSpeed = 110
SlowSpeed = 110
CurveSpeed = 90
StartSpeed = 90
Block = 'U'
P_color = 'U'  # Unknown
P_wall_L = 'N'
P_wall_R = 'N'
LastPillarColor = 'U'
P_x = 0
P_height = 0
LastCurveTime = 0
NextCurveDelay = 5000
walldistance = 40


# Main functions
def ProgramStopUsingGyro():
    global Distance_Front, start_time
    DrivingTime = 0  # driving time
    temporaryTime = 0

    DC.runMotor(SlowSpeed)  # slow down
    # slight countersteering to compensate for overshooting
    if DrivingDirection == 'R':
        ST.left(10)
    else:
        ST.right(10)

    delay(400)
    ST.center()

    # Go straight for at least x msec
    Gyro_steer_straight()
    temporaryTime = millis()
    while millis() < temporaryTime + 1200:
        Gyro_steer_straight()

    # drive straight to finish and stop
    Distance_Front = US.SpaceUltraSonicFront()
    while Distance_Front > 140:
        Gyro_steer_straight()
        Distance_Front = US.SpaceUltraSonicFront()

    DC.stopMotor()
    DC.runMotor_R(SlowSpeed)
    delay(1500)
    DC.stopMotor()

    # save current time in milliseconds
    DrivingTime = millis() - start_time
    L.led_off()
    print(f"Time: {DrivingTime}ms, Distance: {Distance_Front}cm")
    delay(9999999)  # wait forever

def Gyro_steer_straight():
    global angle
    angle = GY.IMU_getAngle()
    Steering = int((angle - StraightAngle) * 0.8)  # 0.8 = wiggle factor
    if Steering > 15.0:
        Steering = 15
    elif Steering < -15.0:
        Steering = -15

    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)

def CalculateTargetDirection():
    if DrivingDirection == 'R':
        return StraightAngle + 90.0
    elif DrivingDirection == 'L':
        return StraightAngle - 90.0
    return 0.0

def CalculateStraightAngle(angle):
    factor = int((angle + 44.0) / 90.0)  # int cuts off fractions
    return factor * 90.0

def TurnLeft():
    global corners, StraightAngle, angle, LastCurveTime, Distance_Left, Distance_Right, Distance_Front
    backtime = 0
    L.led_b()  # Purple equivalent
    ST.center()
    DC.stopMotor()
    Distance_Right = US.SpaceUltraSonicRight()
    
    if Distance_Right > 50:
        backtime = 1500
    else:
        backtime = 200
        
    DC.runMotor(SlowSpeed)
    ST.right(45)

    angle = GY.IMU_getAngle()
    TargetDirection = CalculateTargetDirection()

    DC.runMotor_R(SlowSpeed)  # backwards turn
    while angle > StraightAngle - 45.0:
        angle = GY.IMU_getAngle()
        delay(20)
        
    ST.center()
    delay(1000)
    DC.stopMotor()

    # turn forward
    ST.left(45)
    DC.runMotor(SlowSpeed)
    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
        delay(20)
        
    DC.stopMotor()

    corners += 1
    StraightAngle = TargetDirection
    ST.center()
    DC.stopMotor()
    Distance_Left = US.SpaceUltraSonicLeft()
    Distance_Right = US.SpaceUltraSonicRight()
    Distance_Front = US.SpaceUltraSonicFront()
    DC.runMotor_R(SlowSpeed)
    delay(backtime)
    DC.stopMotor()
    DC.runMotor(SlowSpeed)
    L.led_off()  # White equivalent
    LastCurveTime = millis()
    DC.runMotor(SlowSpeed)

def TurnRight():
    global corners, StraightAngle, angle, LastCurveTime, Distance_Left, Distance_Right, Distance_Front
    backtime = 0
    L.led_b()  # Purple equivalent
    ST.center()
    DC.stopMotor()
    Distance_Left = US.SpaceUltraSonicLeft()
    
    if Distance_Left > 50:
        backtime = 1500
    else:
        backtime = 200
        
    DC.runMotor(SlowSpeed)
    ST.left(45)

    angle = GY.IMU_getAngle()
    TargetDirection = CalculateTargetDirection()

    DC.runMotor_R(SlowSpeed)  # backwards turn
    while angle < StraightAngle + 45.0:
        angle = GY.IMU_getAngle()
        delay(20)
        
    ST.center()
    delay(1000)
    DC.stopMotor()

    # turn forward
    ST.right(45)
    DC.runMotor(SlowSpeed)
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
        delay(20)
        
    DC.stopMotor()

    corners += 1
    StraightAngle = TargetDirection
    ST.center()
    DC.stopMotor()
    Distance_Left = US.SpaceUltraSonicLeft()
    Distance_Right = US.SpaceUltraSonicRight()
    Distance_Front = US.SpaceUltraSonicFront()
    DC.runMotor_R(SlowSpeed)
    delay(backtime)
    DC.stopMotor()
    DC.runMotor(SlowSpeed)
    L.led_off()  # White equivalent
    LastCurveTime = millis()
    DC.runMotor(SlowSpeed)

def alignCenter():
    global Distance_Left, Distance_Right
    Steering = int((Distance_Left - Distance_Right) * 0.3)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)

    delay(20)

def alignLeft():
    global Distance_Left
    Distance_Left = US.SpaceUltraSonicLeft()
    Steering = int((Distance_Left - walldistance) * 0.9)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)

    delay(20)

def alignRight():
    global Distance_Right
    Distance_Right = US.SpaceUltraSonicRight()
    Steering = int((walldistance - Distance_Right) * 0.9)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)

    delay(20)

def CurveLeftUntilBlock():
    global StraightAngle, angle, corners, LastCurveTime
    L.led_b()  # Blue equivalent
    TargetDirection = CalculateTargetDirection()
    print(f"Target Direction: {TargetDirection}")
    
    Distance_Front = US.SpaceUltraSonicFront()
    # go straight to opposite wall
    while Distance_Front > 50:
        Gyro_steer_straight()
        Distance_Front = US.SpaceUltraSonicFront()
        
    # wall reached
    DC.stopMotor()
    # start turning
    ST.left(45)
    DC.runMotor(SlowSpeed)
    angle = GY.IMU_getAngle()
    
    while angle > StraightAngle - 45.0:
        angle = GY.IMU_getAngle()
        
    DC.stopMotor()
    ST.center()
    DC.runMotor(SlowSpeed)
    delay(500)
    DC.stopMotor()
    ST.right(45)
    DC.runMotor_R(SlowSpeed)
    
    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
        
    DC.stopMotor()
    ST.center()
    DC.runMotor(SlowSpeed)
    delay(500)
    corners += 1
    StraightAngle = TargetDirection
    DC.runMotor(SlowSpeed)
    L.led_off()  # White equivalent
    LastCurveTime = millis()

def CurveRightUntilBlock():
    global StraightAngle, angle, corners, LastCurveTime
    L.led_b()  # Blue equivalent
    TargetDirection = CalculateTargetDirection()
    print(f"Target Direction: {TargetDirection}")
    
    Distance_Front = US.SpaceUltraSonicFront()
    # go straight to opposite wall
    while Distance_Front > 50:
        Gyro_steer_straight()
        Distance_Front = US.SpaceUltraSonicFront()
        
    # wall reached
    DC.stopMotor()
    # start turning
    ST.right(45)
    DC.runMotor(SlowSpeed)
    angle = GY.IMU_getAngle()
    
    while angle < StraightAngle + 45.0:
        angle = GY.IMU_getAngle()
        
    DC.stopMotor()
    ST.center()
    DC.runMotor(SlowSpeed)
    delay(500)
    DC.stopMotor()
    ST.left(45)
    DC.runMotor_R(SlowSpeed)
    
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
        
    DC.stopMotor()
    ST.center()
    DC.runMotor(SlowSpeed)
    delay(500)
    corners += 1
    StraightAngle = TargetDirection
    DC.runMotor(SlowSpeed)
    L.led_off()  # White equivalent
    LastCurveTime = millis()
def wallDirectionCheck():
    global DrivingDirection, Distance_Left, Distance_Right
    if Distance_Left < 10:
        DrivingDirection = 'L'
    if Distance_Right < 10:
        DrivingDirection = 'R'

def DriveUntilFirstPillarInLane():
    global P_color, P_x
    DC.runMotor(SlowSpeed)
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    if DrivingDirection == 'L':
        # go straight, until pillar is in sight,
        # ignore pillars on the extreme left, we have already passed them
        while (P_color == 'U') or (P_x < 50):
            Gyro_steer_straight()
            #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    else:
        # go straight, until pillar is in sight,
        # ignore pillars on the extreme right, we have already passed them
        while (P_color == 'U') or (P_x > 270):
            Gyro_steer_straight()
            #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren

def DriveUntilNextCurve():
    global Distance_Front
    Distance_Front = US.SpaceUltraSonicFront()
    while Distance_Front > 100:
        Gyro_steer_straight()
        Distance_Front = US.SpaceUltraSonicFront()

def steerToLaneCenter():
    global Distance_Front, Distance_Left, Distance_Right, angle
    TargetDirection = StraightAngle
    Distance_Front = US.SpaceUltraSonicFront()
    Distance_Left = US.SpaceUltraSonicLeft()
    Distance_Right = US.SpaceUltraSonicRight()

    if Distance_Left < 25:
        # steer to the middle of the lane
        ST.right(25)
        delay(500)
        ST.center()
        delay(500)
        # back to straight
        ST.left(25)

        angle = GY.IMU_getAngle()
        while angle > TargetDirection:
            angle = GY.IMU_getAngle()
            delay(20)
        ST.center()
        DC.runMotor(SlowSpeed)
    elif Distance_Right < 25:
        # steer to the middle of the lane
        ST.left(25)
        delay(500)
        ST.center()
        delay(500)
        # back to straight
        ST.right(25)

        angle = GY.IMU_getAngle()
        while angle > TargetDirection:
            angle = GY.IMU_getAngle()
            delay(20)
        ST.center()
        DC.runMotor(SlowSpeed)
    else:
        # just go straight for 500 msec
        start = millis()
        while millis() < start + 500:
            Gyro_steer_straight()
            delay(20)

def ApproachPillar():
    global P_height, P_color
    DC.runMotor(SlowSpeed)
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    if P_color == 'R':
        L.led_r()  # Red
    elif P_color == 'G':
        L.led_g()  # Green

    while P_height < 25:
        SteerToPillar()
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
        print(f"Pillar height: {P_height}")

def SteerToPillar():
    global P_x
    Steering = int((160 - P_x) * 0.3)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)

def EvadeGreenPillar():
    global angle, StraightAngle, P_x, P_color, LastPillarColor
    L.led_g()  # Green
    angle = GY.IMU_getAngle()
    TargetDirection = StraightAngle
    DC.runMotor(SlowSpeed)
    ST.left(35)
    
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    # turn until block is on the left
    while (P_x < 290) and (P_color == 'G'):
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    
        ST.center()
    delay(200)  # go straight minimum 200 msec

    while P_color == 'G':
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren

        delay(200)
    # back to straight
    ST.right(35)

    angle = GY.IMU_getAngle()
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
        delay(10)
        
    ST.center()
    DC.runMotor(SlowSpeed)
    # drive past
    prev_x = P_x
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    while (P_color == 'G') and (P_x >= prev_x):
        P_x = prev_x
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
        Gyro_steer_straight()

    delay(100)
    L.led_off()  # White equivalent
    LastPillarColor = 'G'

def EvadeRedPillar():
    global angle, StraightAngle, P_x, P_color, LastPillarColor
    L.led_r()  # Red
    angle = GY.IMU_getAngle()
    TargetDirection = StraightAngle
    DC.runMotor(SlowSpeed)
    ST.right(35)
    
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    # turn until block is on the right
    while (P_x > 30) and (P_color == 'R'):
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    
        ST.center()
    delay(200)

    while P_color == 'R':
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren

        delay(200)
    # back to straight
    ST.left(35)

    angle = GY.IMU_getAngle()
    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
        delay(10)
        
    ST.center()
    DC.runMotor(SlowSpeed)
    # drive past
    prev_x = P_x
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    while (P_color == 'R') and (P_x <= prev_x):
        P_x = prev_x
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
        Gyro_steer_straight()

    delay(100)
    L.led_off()  # White equivalent
    LastPillarColor = 'R'

def runCurve():
    global Distance_Right, Distance_Front, Distance_Left
    if DrivingDirection == 'L':
        Distance_Right = US.SpaceUltraSonicRight()
        if Distance_Right < 25:
            CurveLeftUntilBlock()
        else:
            # Go straight to opposite wall
            Distance_Front = US.SpaceUltraSonicFront()
            while Distance_Front > 10:
                Gyro_steer_straight()
                Distance_Front = US.SpaceUltraSonicFront()
            # wall reached
            DC.stopMotor()
            TurnLeft()
    else:
        Distance_Left = US.SpaceUltraSonicLeft()
        if Distance_Left < 25:
            CurveRightUntilBlock()
        else:
            # Go straight to opposite wall
            Distance_Front = US.SpaceUltraSonicFront()
            while Distance_Front > 10:
                Gyro_steer_straight()
                Distance_Front = US.SpaceUltraSonicFront()
            # wall reached
            DC.stopMotor()
            TurnRight()

def runLane():
    global P_color, P_height
    check = False
    DC.stopMotor()
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    print(f"Color: {P_color}, Height: {P_height}")
    delay(200)

    if P_height > 20:
        # lane may have 2 pillars
        DriveUntilFirstPillarInLane()
        ApproachPillar()
        if P_color == 'R':
            EvadeRedPillar()
        else:
            EvadeGreenPillar()
        DC.stopMotor()
        # Another pillar in this lane?
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
        # check if pillar is in lane
        print(f"Color: {P_color}")
        if P_color != 'U':
            check = CheckPillarIsInLane()
        else:
            check = False

        DC.runMotor(SlowSpeed)
        if (P_color == 'R') and (check == True):
            ApproachPillar()
            EvadeRedPillar()
        elif (P_color == 'G') and (check == True):
            ApproachPillar()
            EvadeGreenPillar()
    else:
        # lane has only 1 pillar
        DriveUntilFirstPillarInLane()
        ApproachPillar()
        if P_color == 'R':
            EvadeRedPillar()
        else:
            EvadeGreenPillar()

    DriveUntilNextCurve()

def CheckPillarIsInLane():
    global P_color, P_x
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    if (DrivingDirection == 'L') and (P_color != 'U') and (P_x > 90):
        return True
    elif (DrivingDirection == 'R') and (P_color != 'U') and (P_x < 215):
        return True
    else:
        return False

def measureAllCurrentDistances():
    global Distance_Front, Distance_Left, Distance_Right
    Distance_Front = US.SpaceUltraSonicFront()
    Distance_Left = US.SpaceUltraSonicLeft()
    Distance_Right = US.SpaceUltraSonicRight()

def waitOnButtonPress():
    # Wait for button press
    L.led_y()  # Yellow
    while digitalRead(Button) == LOW:
        delay(50)

def saveCurrentTime():
    global start_time, LastCurveTime
    start_time = millis()
    LastCurveTime = millis() - NextCurveDelay

def evaluateRaspiData():
    global DrivingDirection, P_color, P_x
    if P_color != 'U':
        if P_x < 110:
            DrivingDirection = 'L'
        elif P_x > 210:
            DrivingDirection = 'R'

def startPhase():
    global DrivingDirection, Distance_Front, Distance_Left, Distance_Right, P_color, P_x
    DC.runMotor(SlowSpeed)
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren  # requests what is seen in front
    
    # Check if pillar is right ahead
    if (P_color == 'R') and (P_x < 210) and (P_x > 110):
        ApproachPillar()
        EvadeRedPillar()
        
        DC.runMotor(SlowSpeed)
        while Distance_Front > 10:
            Gyro_steer_straight()
            Distance_Front = US.SpaceUltraSonicFront()
        DC.stopMotor()
        
        if DrivingDirection == 'U':
            Distance_Left = US.SpaceUltraSonicLeft()
            Distance_Right = US.SpaceUltraSonicRight()
            
            if Distance_Left > 150:
                DrivingDirection = 'L'
            elif Distance_Right > 150:
                DrivingDirection = 'R'
            else:
                # Try with direction of next pillar
                DC.runMotor_R(SlowSpeed)
                delay(800)
                DC.stopMotor()
                #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
                
                if P_x > 160:
                    DrivingDirection = 'R'
                else:
                    DrivingDirection = 'L'
                    
                Distance_Front = US.SpaceUltraSonicFront()
                DC.runMotor(SlowSpeed)
                
                while Distance_Front > 10:
                    Gyro_steer_straight()
                    Distance_Front = US.SpaceUltraSonicFront()
                DC.stopMotor()
                
    elif (P_color == 'G') and (P_x < 210) and (P_x > 110):
        ApproachPillar()
        EvadeGreenPillar()
        
        DC.runMotor(SlowSpeed)
        Distance_Front = US.SpaceUltraSonicFront()
        
        while Distance_Front > 10:
            Gyro_steer_straight()
            Distance_Front = US.SpaceUltraSonicFront()
        DC.stopMotor()
        
        if DrivingDirection == 'U':
            Distance_Left = US.SpaceUltraSonicLeft()
            Distance_Right = US.SpaceUltraSonicRight()
            
            if Distance_Left > 150:
                DrivingDirection = 'L'
            elif Distance_Right > 150:
                DrivingDirection = 'R'
            else:
                # Try with direction of next pillar
                DC.runMotor_R(SlowSpeed)
                delay(800)
                DC.stopMotor()
                #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
                
                if P_x > 160:
                    DrivingDirection = 'R'
                else:
                    DrivingDirection = 'L'
                    
                Distance_Front = US.SpaceUltraSonicFront()
                DC.runMotor(SlowSpeed)
                
                while Distance_Front > 10:
                    Gyro_steer_straight()
                    Distance_Front = US.SpaceUltraSonicFront()
                DC.stopMotor()
                
    else:
        # No pillar directly ahead
        while Distance_Front > 10:
            Gyro_steer_straight()
            Distance_Front = US.SpaceUltraSonicFront()
            
        # Wall reached
        DC.stopMotor()
        
        # If driving direction is unclear, find out
        if DrivingDirection == 'U':
            Distance_Left = US.SpaceUltraSonicLeft()
            Distance_Right = US.SpaceUltraSonicRight()
            
            if Distance_Left > 80:
                DrivingDirection = 'L'
            elif Distance_Right > 80:
                DrivingDirection = 'R'
            else:
                if Distance_Left > Distance_Right:
                    DrivingDirection = 'L'
                else:
                    DrivingDirection = 'R'

    DC.stopMotor()
    print(f"Direction: {DrivingDirection}, Left: {Distance_Left}, Right: {Distance_Right}")
    delay(500)
    DC.runMotor(SlowSpeed)

    # Turn
    if DrivingDirection == 'L':
        if Distance_Right < 25:
            Distance_Front = US.SpaceUltraSonicFront()
            DC.runMotor_R(SlowSpeed)
            
            while Distance_Front < 50:
                Distance_Front = US.SpaceUltraSonicFront()
            DC.stopMotor()
            DC.runMotor(SlowSpeed)
            CurveLeftUntilBlock()
        else:
            TurnLeft()
    else:
        if Distance_Left < 25:
            Distance_Front = US.SpaceUltraSonicFront()

# Main functions
def ProgramStopUsingGyro():
    global Distance_Front, start_time
    DrivingTime = 0  # driving time
    temporaryTime = 0

    DC.runMotor(SlowSpeed)  # slow down
    # slight countersteering to compensate for overshooting
    if DrivingDirection == 'R':
        ST.left(10)
    else:
        ST.right(10)

    delay(400)
    ST.center()

    # Go straight for at least x msec
    Gyro_steer_straight()
    temporaryTime = millis()
    while millis() < temporaryTime + 1200:
        Gyro_steer_straight()

    # drive straight to finish and stop
    Distance_Front = US.SpaceUltraSonicFront()
    while Distance_Front > 140:
        Gyro_steer_straight()
        Distance_Front = US.SpaceUltraSonicFront()

    DC.stopMotor()
    DC.runMotor_R(SlowSpeed)
    delay(1500)
    DC.stopMotor()

    # save current time in milliseconds
    DrivingTime = millis() - start_time
    L.led_off()
    print(f"Time: {DrivingTime}ms, Distance: {Distance_Front}cm")
    delay(9999999)  # wait forever

def Gyro_steer_straight():
    global angle
    angle = GY.IMU_getAngle()
    Steering = int((angle - StraightAngle) * 0.8)  # 0.8 = wiggle factor
    if Steering > 15.0:
        Steering = 15
    elif Steering < -15.0:
        Steering = -15

    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)

def CalculateTargetDirection():
    if DrivingDirection == 'R':
        return StraightAngle + 90.0
    elif DrivingDirection == 'L':
        return StraightAngle - 90.0
    return 0.0

def CalculateStraightAngle(angle):
    factor = int((angle + 44.0) / 90.0)  # int cuts off fractions
    return factor * 90.0

def TurnLeft():
    global corners, StraightAngle, angle, LastCurveTime, Distance_Left, Distance_Right, Distance_Front
    backtime = 0
    L.led_b()  # Purple equivalent
    ST.center()
    DC.stopMotor()
    Distance_Right = US.SpaceUltraSonicRight()
    
    if Distance_Right > 50:
        backtime = 1500
    else:
        backtime = 200
        
    DC.runMotor(SlowSpeed)
    ST.right(45)

    angle = GY.IMU_getAngle()
    TargetDirection = CalculateTargetDirection()

    DC.runMotor_R(SlowSpeed)  # backwards turn
    while angle > StraightAngle - 45.0:
        angle = GY.IMU_getAngle()
        delay(20)
        
    ST.center()
    delay(1000)
    DC.stopMotor()

    # turn forward
    ST.left(45)
    DC.runMotor(SlowSpeed)
    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
        delay(20)
        
    DC.stopMotor()

    corners += 1
    StraightAngle = TargetDirection
    ST.center()
    DC.stopMotor()
    Distance_Left = US.SpaceUltraSonicLeft()
    Distance_Right = US.SpaceUltraSonicRight()
    Distance_Front = US.SpaceUltraSonicFront()
    DC.runMotor_R(SlowSpeed)
    delay(backtime)
    DC.stopMotor()
    DC.runMotor(SlowSpeed)
    L.led_off()  # White equivalent
    LastCurveTime = millis()
    DC.runMotor(SlowSpeed)

def TurnRight():
    global corners, StraightAngle, angle, LastCurveTime, Distance_Left, Distance_Right, Distance_Front
    backtime = 0
    L.led_b()  # Purple equivalent
    ST.center()
    DC.stopMotor()
    Distance_Left = US.SpaceUltraSonicLeft()
    
    if Distance_Left > 50:
        backtime = 1500
    else:
        backtime = 200
        
    DC.runMotor(SlowSpeed)
    ST.left(45)

    angle = GY.IMU_getAngle()
    TargetDirection = CalculateTargetDirection()

    DC.runMotor_R(SlowSpeed)  # backwards turn
    while angle < StraightAngle + 45.0:
        angle = GY.IMU_getAngle()
        delay(20)
        
    ST.center()
    delay(1000)
    DC.stopMotor()

    # turn forward
    ST.right(45)
    DC.runMotor(SlowSpeed)
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
        delay(20)
        
    DC.stopMotor()

    corners += 1
    StraightAngle = TargetDirection
    ST.center()
    DC.stopMotor()
    Distance_Left = US.SpaceUltraSonicLeft()
    Distance_Right = US.SpaceUltraSonicRight()
    Distance_Front = US.SpaceUltraSonicFront()
    DC.runMotor_R(SlowSpeed)
    delay(backtime)
    DC.stopMotor()
    DC.runMotor(SlowSpeed)
    L.led_off()  # White equivalent
    LastCurveTime = millis()
    DC.runMotor(SlowSpeed)

def alignCenter():
    global Distance_Left, Distance_Right
    Steering = int((Distance_Left - Distance_Right) * 0.3)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)

    delay(20)

def alignLeft():
    global Distance_Left
    Distance_Left = US.SpaceUltraSonicLeft()
    Steering = int((Distance_Left - walldistance) * 0.9)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)

    delay(20)

def alignRight():
    global Distance_Right
    Distance_Right = US.SpaceUltraSonicRight()
    Steering = int((walldistance - Distance_Right) * 0.9)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)

    delay(20)

def CurveLeftUntilBlock():
    global StraightAngle, angle, corners, LastCurveTime
    L.led_b()  # Blue equivalent
    TargetDirection = CalculateTargetDirection()
    print(f"Target Direction: {TargetDirection}")
    
    Distance_Front = US.SpaceUltraSonicFront()
    # go straight to opposite wall
    while Distance_Front > 50:
        Gyro_steer_straight()
        Distance_Front = US.SpaceUltraSonicFront()
        
    # wall reached
    DC.stopMotor()
    # start turning
    ST.left(45)
    DC.runMotor(SlowSpeed)
    angle = GY.IMU_getAngle()
    
    while angle > StraightAngle - 45.0:
        angle = GY.IMU_getAngle()
        
    DC.stopMotor()
    ST.center()
    DC.runMotor(SlowSpeed)
    delay(500)
    DC.stopMotor()
    ST.right(45)
    DC.runMotor_R(SlowSpeed)
    
    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
        
    DC.stopMotor()
    ST.center()
    DC.runMotor(SlowSpeed)
    delay(500)
    corners += 1
    StraightAngle = TargetDirection
    DC.runMotor(SlowSpeed)
    L.led_off()  # White equivalent
    LastCurveTime = millis()

def CurveRightUntilBlock():
    global StraightAngle, angle, corners, LastCurveTime
    L.led_b()  # Blue equivalent
    TargetDirection = CalculateTargetDirection()
    print(f"Target Direction: {TargetDirection}")
    
    Distance_Front = US.SpaceUltraSonicFront()
    # go straight to opposite wall
    while Distance_Front > 50:
        Gyro_steer_straight()
        Distance_Front = US.SpaceUltraSonicFront()
        
    # wall reached
    DC.stopMotor()
    # start turning
    ST.right(45)
    DC.runMotor(SlowSpeed)
    angle = GY.IMU_getAngle()
    
    while angle < StraightAngle + 45.0:
        angle = GY.IMU_getAngle()
        
    DC.stopMotor()
    ST.center()
    DC.runMotor(SlowSpeed)
    delay(500)
    DC.stopMotor()
    ST.left(45)
    DC.runMotor_R(SlowSpeed)
    
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
        
    DC.stopMotor()
    ST.center()
    DC.runMotor(SlowSpeed)
    delay(500)
    corners += 1
    StraightAngle = TargetDirection
    DC.runMotor(SlowSpeed)
    L.led_off()  # White equivalent
    LastCurveTime = millis()

def DriveUntilFirstPillarInLane():
    global P_color, P_x
    DC.runMotor(SlowSpeed)
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    if DrivingDirection == 'L':
        # go straight, until pillar is in sight,
        # ignore pillars on the extreme left, we have already passed them
        while (P_color == 'U') or (P_x < 50):
            Gyro_steer_straight()
            #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    else:
        # go straight, until pillar is in sight,
        # ignore pillars on the extreme right, we have already passed them
        while (P_color == 'U') or (P_x > 270):
            Gyro_steer_straight()
            #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren

def DriveUntilNextCurve():
    global Distance_Front
    Distance_Front = US.SpaceUltraSonicFront()
    while Distance_Front > 100:
        Gyro_steer_straight()
        Distance_Front = US.SpaceUltraSonicFront()

def steerToLaneCenter():
    global Distance_Front, Distance_Left, Distance_Right, angle
    TargetDirection = StraightAngle
    Distance_Front = US.SpaceUltraSonicFront()
    Distance_Left = US.SpaceUltraSonicLeft()
    Distance_Right = US.SpaceUltraSonicRight()

    if Distance_Left < 25:
        # steer to the middle of the lane
        ST.right(25)
        delay(500)
        ST.center()
        delay(500)
        # back to straight
        ST.left(25)

        angle = GY.IMU_getAngle()
        while angle > TargetDirection:
            angle = GY.IMU_getAngle()
            delay(20)
        ST.center()
        DC.runMotor(SlowSpeed)
    elif Distance_Right < 25:
        # steer to the middle of the lane
        ST.left(25)
        delay(500)
        ST.center()
        delay(500)
        # back to straight
        ST.right(25)

        angle = GY.IMU_getAngle()
        while angle > TargetDirection:
            angle = GY.IMU_getAngle()
            delay(20)
        ST.center()
        DC.runMotor(SlowSpeed)
    else:
        # just go straight for 500 msec
        start = millis()
        while millis() < start + 500:
            Gyro_steer_straight()
            delay(20)

def ApproachPillar():
    global P_height, P_color
    DC.runMotor(SlowSpeed)
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    if P_color == 'R':
        L.led_r()  # Red
    elif P_color == 'G':
        L.led_g()  # Green

    while P_height < 25:
        SteerToPillar()
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
        print(f"Pillar height: {P_height}")

def SteerToPillar():
    global P_x
    Steering = int((160 - P_x) * 0.3)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)

def EvadeGreenPillar():
    global angle, StraightAngle, P_x, P_color, LastPillarColor
    L.led_g()  # Green
    angle = GY.IMU_getAngle()
    TargetDirection = StraightAngle
    DC.runMotor(SlowSpeed)
    ST.left(35)
    
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    # turn until block is on the left
    while (P_x < 290) and (P_color == 'G'):
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    
        ST.center()
    delay(200)  # go straight minimum 200 msec

    while P_color == 'G':
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren

        delay(200)
    # back to straight
    ST.right(35)

    angle = GY.IMU_getAngle()
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
        delay(10)
        
    ST.center()
    DC.runMotor(SlowSpeed)
    # drive past
    prev_x = P_x
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    while (P_color == 'G') and (P_x >= prev_x):
        P_x = prev_x
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
        Gyro_steer_straight()

    delay(100)
    L.led_off()  # White equivalent
    LastPillarColor = 'G'

def EvadeRedPillar():
    global angle, StraightAngle, P_x, P_color, LastPillarColor
    L.led_r()  # Red
    angle = GY.IMU_getAngle()
    TargetDirection = StraightAngle
    DC.runMotor(SlowSpeed)
    ST.right(35)
    
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    # turn until block is on the right
    while (P_x > 30) and (P_color == 'R'):
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    
        ST.center()
    delay(200)

    while P_color == 'R':
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren

        delay(200)
    # back to straight
    ST.left(35)

    angle = GY.IMU_getAngle()
    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
        delay(10)
        
    ST.center()
    DC.runMotor(SlowSpeed)
    # drive past
    prev_x = P_x
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    while (P_color == 'R') and (P_x <= prev_x):
        P_x = prev_x
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
        Gyro_steer_straight()

    delay(100)
    L.led_off()  # White equivalent
    LastPillarColor = 'R'

def runCurve():
    global Distance_Right, Distance_Front, Distance_Left
    if DrivingDirection == 'L':
        Distance_Right = US.SpaceUltraSonicRight()
        if Distance_Right < 25:
            CurveLeftUntilBlock()
        else:
            # Go straight to opposite wall
            Distance_Front = US.SpaceUltraSonicFront()
            while Distance_Front > 10:
                Gyro_steer_straight()
                Distance_Front = US.SpaceUltraSonicFront()
            # wall reached
            DC.stopMotor()
            TurnLeft()
    else:
        Distance_Left = US.SpaceUltraSonicLeft()
        if Distance_Left < 25:
            CurveRightUntilBlock()
        else:
            # Go straight to opposite wall
            Distance_Front = US.SpaceUltraSonicFront()
            while Distance_Front > 10:
                Gyro_steer_straight()
                Distance_Front = US.SpaceUltraSonicFront()
            # wall reached
            DC.stopMotor()
            TurnRight()

def runLane():
    global P_color, P_height
    check = False
    DC.stopMotor()
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    print(f"Color: {P_color}, Height: {P_height}")
    delay(200)

    if P_height > 20:
        # lane may have 2 pillars
        DriveUntilFirstPillarInLane()
        ApproachPillar()
        if P_color == 'R':
            EvadeRedPillar()
        else:
            EvadeGreenPillar()
        DC.stopMotor()
        # Another pillar in this lane?
        #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
        # check if pillar is in lane
        print(f"Color: {P_color}")
        if P_color != 'U':
            check = CheckPillarIsInLane()
        else:
            check = False

        DC.runMotor(SlowSpeed)
        if (P_color == 'R') and (check == True):
            ApproachPillar()
            EvadeRedPillar()
        elif (P_color == 'G') and (check == True):
            ApproachPillar()
            EvadeGreenPillar()
    else:
        # lane has only 1 pillar
        DriveUntilFirstPillarInLane()
        ApproachPillar()
        if P_color == 'R':
            EvadeRedPillar()
        else:
            EvadeGreenPillar()

    DriveUntilNextCurve()

def CheckPillarIsInLane():
    global P_color, P_x
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
    if (DrivingDirection == 'L') and (P_color != 'U') and (P_x > 90):
        return True
    elif (DrivingDirection == 'R') and (P_color != 'U') and (P_x < 215):
        return True
    else:
        return False

def measureAllCurrentDistances():
    global Distance_Front, Distance_Left, Distance_Right
    Distance_Front = US.SpaceUltraSonicFront()
    Distance_Left = US.SpaceUltraSonicLeft()
    Distance_Right = US.SpaceUltraSonicRight()

def waitOnButtonPress():
    # Wait for button press
    L.led_y()  # Yellow
    while digitalRead(Button) == LOW:
        delay(50)

def saveCurrentTime():
    global start_time, LastCurveTime
    start_time = millis()
    LastCurveTime = millis() - NextCurveDelay

def evaluateRaspiData():
    global DrivingDirection, P_color, P_x
    if P_color != 'U':
        if P_x < 110:
            DrivingDirection = 'L'
        elif P_x > 210:
            DrivingDirection = 'R'

def startPhase():
    global DrivingDirection, Distance_Front, Distance_Left, Distance_Right, P_color, P_x
    DC.runMotor(SlowSpeed)
    #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren  # requests what is seen in front
    
    # Check if pillar is right ahead
    if (P_color == 'R') and (P_x < 210) and (P_x > 110):
        ApproachPillar()
        EvadeRedPillar()
        
        DC.runMotor(SlowSpeed)
        while Distance_Front > 10:
            Gyro_steer_straight()
            Distance_Front = US.SpaceUltraSonicFront()
        DC.stopMotor()
        
        if DrivingDirection == 'U':
            Distance_Left = US.SpaceUltraSonicLeft()
            Distance_Right = US.SpaceUltraSonicRight()
            
            if Distance_Left > 150:
                DrivingDirection = 'L'
            elif Distance_Right > 150:
                DrivingDirection = 'R'
            else:
                # Try with direction of next pillar
                DC.runMotor_R(SlowSpeed)
                delay(800)
                DC.stopMotor()
                #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
                
                if P_x > 160:
                    DrivingDirection = 'R'
                else:
                    DrivingDirection = 'L'
                    
                Distance_Front = US.SpaceUltraSonicFront()
                DC.runMotor(SlowSpeed)
                
                while Distance_Front > 10:
                    Gyro_steer_straight()
                    Distance_Front = US.SpaceUltraSonicFront()
                DC.stopMotor()
                
    elif (P_color == 'G') and (P_x < 210) and (P_x > 110):
        ApproachPillar()
        EvadeGreenPillar()
        
        DC.runMotor(SlowSpeed)
        Distance_Front = US.SpaceUltraSonicFront()
        
        while Distance_Front > 10:
            Gyro_steer_straight()
            Distance_Front = US.SpaceUltraSonicFront()
        DC.stopMotor()
        
        if DrivingDirection == 'U':
            Distance_Left = US.SpaceUltraSonicLeft()
            Distance_Right = US.SpaceUltraSonicRight()
            
            if Distance_Left > 150:
                DrivingDirection = 'L'
            elif Distance_Right > 150:
                DrivingDirection = 'R'
            else:
                # Try with direction of next pillar
                DC.runMotor_R(SlowSpeed)
                delay(800)
                DC.stopMotor()
                #findNextPillar() keine aktuelle definiton fuer die funktion muss naechstes mal definieren
                
                if P_x > 160:
                    DrivingDirection = 'R'
                else:
                    DrivingDirection = 'L'
                    
                Distance_Front = US.SpaceUltraSonicFront()
                DC.runMotor(SlowSpeed)
                
                while Distance_Front > 10:
                    Gyro_steer_straight()
                    Distance_Front = US.SpaceUltraSonicFront()
                DC.stopMotor()
                
    else:
        # No pillar directly ahead
        while Distance_Front > 10:
            Gyro_steer_straight()
            Distance_Front = US.SpaceUltraSonicFront()
            
        # Wall reached
        DC.stopMotor()
        
        # If driving direction is unclear, find out
        if DrivingDirection == 'U':
            Distance_Left = US.SpaceUltraSonicLeft()
            Distance_Right = US.SpaceUltraSonicRight()
            
            if Distance_Left > 80:
                DrivingDirection = 'L'
            elif Distance_Right > 80:
                DrivingDirection = 'R'
            else:
                if Distance_Left > Distance_Right:
                    DrivingDirection = 'L'
                else:
                    DrivingDirection = 'R'

    DC.stopMotor()
    print(f"Direction: {DrivingDirection}, Left: {Distance_Left}, Right: {Distance_Right}")
    delay(500)
    DC.runMotor(SlowSpeed)

    # Turn
    if DrivingDirection == 'L':
        if Distance_Right < 25:
            Distance_Front = US.SpaceUltraSonicFront()
            DC.runMotor_R(SlowSpeed)
            
            while Distance_Front < 50:
                Distance_Front = US.SpaceUltraSonicFront()
            DC.stopMotor()
            DC.runMotor(SlowSpeed)
            CurveLeftUntilBlock()
        else:
            TurnLeft()
    else:
        if Distance_Left < 25:
            Distance_Front = US.SpaceUltraSonicFront()
            DC.runMotor_R(SlowSpeed)
            while (Distance_Front < 50):
                Distance_Front = US.SpaceUltraSonicFront()
            DC.stopMotor()
            DC.runMotor(SlowSpeed)
            CurveRightUntilBlock()
        else:
            TurnRight()


def LaneWithUTurn():
    check = False
    DriveUntilFirstPillarInLane()
    ApproachPillar()
    if (P_color == 'R'):
        EvadeRedPillar()
    else:
        EvadeGreenPillar()
    
    if (LastPillarColor == 'R'):
        L.led_r()  # Red for Right/red obstacle
        UTurn()
        if (DrivingDirection == 'R'):
            DrivingDirection = 'L'
        else:
            DrivingDirection = 'R'
    else:
        pass

    ()
    if (P_color != 'U'):
        check = CheckPillarIsInLane()
    else:
        check = False
    
    DC.runMotor(SlowSpeed)
    if ((P_color == 'R') and (check == True)):
        ApproachPillar()
        EvadeRedPillar()
    elif ((P_color == 'G') and (check == True)):
        ApproachPillar()
        EvadeGreenPillar()
    else:
        DriveUntilNextCurve()


def UTurnRight():
    TargetDirection = 0.0
    angle = 0.0
    DC.runMotor(SlowSpeed)
    TargetDirection = StraightAngle + 45.0
    ST.right(45)
    angle = GY.IMU_getAngle()
    ST.runMotor(SlowSpeed)

    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()
    ST.left(45)
    TargetDirection = StraightAngle + 90.0
    DC.runMotor_R(SlowSpeed)
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()

    TargetDirection = StraightAngle + 135.0
    ST.right(45)
    angle = GY.IMU_getAngle()
    DC.runMotor(SlowSpeed)

    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()
    ST.left(45)
    TargetDirection = StraightAngle + 180.0
    DC.runMotor_R(SlowSpeed)
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()

    ST.center()

    DC.runMotor(SlowSpeed)

    return TargetDirection

def UTurnLeft():
    TargetDirection = 0.0
    angle = 0.0
    DC.runMotor(SlowSpeed)
    TargetDirection = StraightAngle - 45.0
    ST.left(45)
    angle = GY.IMU_getAngle()
    DC.runMotor(SlowSpeed)

    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()
    ST.right(45)
    TargetDirection = StraightAngle - 90.0
    DC.runMotor_R(SlowSpeed)
    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()

    TargetDirection = StraightAngle - 135.0
    ST.left(45)
    angle = GY.IMU_getAngle()
    DC.runMotor(SlowSpeed)

    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()
    ST.right(45)
    TargetDirection = StraightAngle - 180.0
    DC.runMotor_R(SlowSpeed)
    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()

    ST.center()

    DC.runMotor(SlowSpeed)

    return TargetDirection

def UTurn():
    TargetDirection = 0.0
    DC.runMotor(SlowSpeed)
    time.sleep(1)  # Assuming delay is in seconds
    DC.stopMotor()
    Distance_Left = US.SpaceUltraSonicLeft()
    Distance_Right = US.SpaceUltraSonicRight()
    
    if Distance_Left > Distance_Right:
        TargetDirection = UTurnLeft()
    else:
        TargetDirection = UTurnRight()
    
    StraightAngle = TargetDirection

if __name__ == '__main__':
    try:
        # fix main function later

        while True:
            ...
    except Exception as e:
        print(e)
