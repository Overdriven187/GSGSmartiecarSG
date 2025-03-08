import time
from Tools import Led as L
from libs import DCMotor as DC
from libs import steering as ST
from libs import gyro as GY
# Global variables and simulation parameters
prog_start_time = time.time()

DC.stopMotor()
DrivingDirection = 'U'
SlowSpeed = 50
CurveSpeed = 100
NormalSpeed = 70
correction_Left = 5.0
correction_Right = 5.0
Walldistance = 30
NextCurveDelay = 1000  # in milliseconds

Distance_Front = 150  # initial simulated distance in cm
Distance_Left = 100   # initial simulated distance in cm
Distance_Right = 100  # initial simulated distance in cm

StraightAngle = 0.0
corners = 0
start_time = 0
LastCurveTime = 0
angle = 0.0
current_angle = 0.0

# Simulated sensor distances (for ultrasonic sensors)
sensor_front = 150
sensor_left = 100
sensor_right = 100

# Simulated button pin
Button = 2
LOW = 0
HIGH = 1

# Delay function: delay in milliseconds
def delay(ms):
    time.sleep(ms / 1000.0)

# millis() function: returns elapsed milliseconds since program start
def millis():
    return int((time.time() - prog_start_time) * 1000)

# Dummy implementation of the ultrasonic sensor functions
def SpaceUltraSonicFront():
    global sensor_front
    # Simulate sensor reading decreasing gradually if above threshold 140
    if sensor_front > 140:
        sensor_front -= 1
    return sensor_front

def SpaceUltraSonicLeft():
    global sensor_left
    # Simulate sensor reading decreasing gradually if above 60
    if sensor_left > 60:
        sensor_left -= 1
    return sensor_left

def SpaceUltraSonicRight():
    global sensor_right
    # Simulate sensor reading decreasing gradually if above 60
    if sensor_right > 60:
        sensor_right -= 1
    return sensor_right

# Dummy digitalRead implementation for the button
def digitalRead(pin):
    # For simulation, always return HIGH (button pressed)
    return HIGH


# ProgramStopUsingGyro()
# stops the car using the gyrovalues saved from beginning

def ProgramStopUsingGyro():
    global Distance_Front, start_time
    DrivingTime = 0  # driving time
    temporaryTime = 0

    runMotor(SlowSpeed)  # slow down
    # slight countersteering to compensate for overshooting
    if DrivingDirection == 'R':
        left(10)
    else:
        right(10)

    delay(400)
    center()

    # Go straight for at least x msec
    Gyro_steer_straight()
    temporaryTime = millis()
    while millis() < temporaryTime + 1200:
        Gyro_steer_straight()

    # drive straight to finish and stop
    global Distance_Front
    Distance_Front = SpaceUltraSonicFront()
    while Distance_Front > 140:
        Gyro_steer_straight()
        Distance_Front = SpaceUltraSonicFront()

    stopMotor()
    runMotor_R(SlowSpeed)
    delay(1500)
    stopMotor()

    # save current time in milliseconds
    DrivingTime = millis() - start_time
    L.led_off()
    #lcd.setCursor(0, 0)
    delay(9999999)  # wait forever

 

# StartNarrow_L()
# startnarrow Left (start close to left inside wall)

def StartNarrow_L():
    global current_angle
    angle = 0.0
    TargetDirection = 360.0
    Speed = SlowSpeed
    right(30)
    delay(500)
    center()
    delay(500)
    angle = IMU_getAngle()
    #lcd.setCursor(0, 0)
    left(30)
    while angle > TargetDirection + correction_Left:
        angle = IMU_getAngle()
        Speed = Speed + 5
        if Speed > CurveSpeed:
            Speed = CurveSpeed
        runMotor(Speed)
        delay(50)
    center()
    runMotor(SlowSpeed)
 

# StartNarrow_R()
# startnarrow Right (start close to right inside wall)

def StartNarrow_R():
    global current_angle
    angle = 0.0
    TargetDirection = 0.0
    Speed = SlowSpeed  # Initialize Speed
    left(30)
    delay(500)
    center()
    delay(500)
    angle = IMU_getAngle()
    #lcd.setCursor(0, 0)
    right(30)
    while angle < TargetDirection - correction_Right:
        angle = IMU_getAngle()
        Speed = Speed + 5
        if Speed > CurveSpeed:
            Speed = CurveSpeed
        runMotor(Speed)
        delay(50)
    center()
    runMotor(SlowSpeed)
 

# alignCenter()
# aligns to the center of the track

def alignCenter():
    global Distance_Left, Distance_Right
    Steering = int((Distance_Left - Distance_Right) * 0.3)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        right(Steering)
    else:
        left(Steering)

    delay(20)
 

# alignLeft()
# aligns to the left wall

def alignLeft():
    global Distance_Left
    Distance_Left = SpaceUltraSonicLeft()
    Steering = int((Distance_Left - Walldistance) * 0.9)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        right(Steering)
    else:
        left(Steering)

    delay(20)
 

# alignRight()
# aligns to the right wall

def alignRight():
    global Distance_Right
    Distance_Right = SpaceUltraSonicRight()
    Steering = int((Walldistance - Distance_Right) * 0.9)
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30

    if Steering < 0:
        Steering = Steering * (-1)
        right(Steering)
    else:
        left(Steering)

    delay(20)
 

# Gyro_steer_straight()
# uses the gyro to orient itself straight to the track
##################################
def Gyro_steer_straight():
    angle = IMU_getAngle()
    Steering = int((angle - StraightAngle) * 0.8)  # 0.8 = wiggle factor
    if Steering > 15.0:
        Steering = 15
    elif Steering < -15.0:
        Steering = -15

    if Steering < 0:
        Steering = Steering * (-1)
        right(Steering)
    else:
        left(Steering)

    # delay(20)
 

# Curve_L()
# calls a left curve and counts it

def Curve_L():
    global corners, StraightAngle, angle, LastCurveTime, Distance_Left, sensor_left, current_angle
    Speed = CurveSpeed
    left(40)
    #lcd.setRGB(0, 0, 255)
    TargetDirection = StraightAngle - 90.0
    angle = IMU_getAngle()
    StraightAngle = TargetDirection
    runMotor(SlowSpeed)
    while angle > TargetDirection + correction_Left:
        angle = IMU_getAngle()
        Speed = Speed + 5
        if Speed > CurveSpeed:
            Speed = CurveSpeed
        runMotor(Speed)
        delay(50)
    corners = corners + 1
    StraightAngle = TargetDirection
    center()
    runMotor(NormalSpeed)
    Distance_Left = SpaceUltraSonicLeft()
    # try to find the inside wall again
    while Distance_Left > 60:
        Distance_Left = SpaceUltraSonicLeft()
        Gyro_steer_straight()
    #lcd.setRGB(0, 255, 0)
    LastCurveTime = millis()
 

# Curve_R()
# calls a right curve and counts it

def Curve_R():
    global corners, StraightAngle, angle, LastCurveTime, Distance_Right, sensor_right, current_angle
    Speed = CurveSpeed
    right(40)
    #lcd.setRGB(0, 0, 255)
    TargetDirection = StraightAngle + 90.0
    angle = IMU_getAngle()
    StraightAngle = TargetDirection
    runMotor(SlowSpeed)
    # checks if the car has a greater angle than the target direction in order to correct it
    while angle < TargetDirection - correction_Right:
        angle = IMU_getAngle()
        Speed = Speed + 5
        if Speed > CurveSpeed:
            Speed = CurveSpeed
        runMotor(Speed)
        delay(50)
    corners = corners + 1
    StraightAngle = TargetDirection
    center()
    runMotor(NormalSpeed)
    Distance_Right = SpaceUltraSonicRight()
    # try to find the inside wall again
    while Distance_Right > 60:
        Distance_Right = SpaceUltraSonicRight()
        Gyro_steer_straight()
    #lcd.setRGB(0, 255, 0)
    LastCurveTime = millis()
 

# measureAllDistances()
# measures all distances and saves them in global variables

def measureAllCurrentDistances():
    global Distance_Front, Distance_Left, Distance_Right
    Distance_Front = SpaceUltraSonicFront()
    Distance_Left = SpaceUltraSonicLeft()
    Distance_Right = SpaceUltraSonicRight() 

# waitOnButtonPress()
# waits until the button is pressed

def waitOnButtonPress():
    while digitalRead(Button) == LOW:
        delay(50)
 

# slowspeedToFindCorrectDirection()
# finds the driving direction slowly while measuring the distances

def slowspeedToFindCorrectDirection():
    global DrivingDirection, Distance_Left, Distance_Right
    if DrivingDirection == 'U':
        runMotor(SlowSpeed)
        while (Distance_Left < 80.0) and (Distance_Right < 80.0):
            Distance_Left = SpaceUltraSonicLeft()
            Distance_Right = SpaceUltraSonicRight()
            Gyro_steer_straight()
        if Distance_Left >= 80.0:
            DrivingDirection = 'L'
            Curve_L()
        elif Distance_Right >= 80.0:
            DrivingDirection = 'R'
            Curve_R()
    elif DrivingDirection == 'R':
        runMotor(SlowSpeed)
        delay(100)
        StartNarrow_R()
        Distance_Right = SpaceUltraSonicRight()
        while Distance_Right < 80.0:
            Distance_Right = SpaceUltraSonicRight()
            Gyro_steer_straight()
        Curve_R()
    elif DrivingDirection == 'L':
        runMotor(SlowSpeed)
        delay(100)
        StartNarrow_L()
        Distance_Left = SpaceUltraSonicLeft()
        while Distance_Left < 80.0:
            Distance_Left = SpaceUltraSonicLeft()
            Gyro_steer_straight()
        Curve_L()
 

# saveCurrentTime()
# saves the current time in milliseconds subtracting NextCurveDelay

def saveCurrentTime():
    global start_time, LastCurveTime
    start_time = millis()
    LastCurveTime = millis() - NextCurveDelay
 

# wallDirectionCheck()
# checks if the car is closer to the left or right wall

def wallDirectionCheck():
    global DrivingDirection, Distance_Left, Distance_Right
    if Distance_Left < 10:
        DrivingDirection = 'L'
    if Distance_Right < 10:
        DrivingDirection = 'R'
 

# operateNavigationThroughTurns()
# uses the gyro to navigate through the turns while counting corners

def operateNavigationThroughTurns():
    global corners, Distance_Right, Distance_Left, LastCurveTime
    if DrivingDirection == 'R':
        while corners < 12:
            # clockwise running for right
            Distance_Right = SpaceUltraSonicRight()
            # check for Right Turn
            if (Distance_Right > 80) and (millis() - LastCurveTime >= NextCurveDelay):
                Curve_R()
            else:
                alignRight()
    else:
        while corners < 12:
            # counterclockwise running for left
            Distance_Left = SpaceUltraSonicLeft()
            # check for Left Turn
            if (Distance_Left > 80) and (millis() - LastCurveTime >= NextCurveDelay):
                Curve_L()
            else:
                alignLeft()
                

if __name__ == "__main__":
    try:
        #setup
        L.led_r()
        delay(5000)
        L.led_y()
        waitOnButtonPress()
        measureAllCurrentDistances()
        wallDirectionCheck()
        StraightAngle = IMU_getAngle()
        
        #loop
        while True:
        
    
    except 