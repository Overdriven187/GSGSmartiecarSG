# GSG Smartiecar SG - Source Code Documentation

## Table of Contents
1. [Overview](#overview)
2. [Project Structure](#project-structure)
3. [Main Programs](#main-programs)
4. [Library Modules](#library-modules)
5. [Testing & Utility Tools](#testing--utility-tools)
6. [Hardware Integration](#hardware-integration)
7. [Communication Patterns](#communication-patterns)
8. [Complex Functions Explained](#complex-functions-explained)
9. [Setup and Dependencies](#setup-and-dependencies)

---

## Overview

This is an autonomous robot car (Smartiecar) project designed for robotics competitions. The system is built to run on a Raspberry Pi and uses various sensors and actuators to navigate through obstacle courses and racing tracks. The robot can:

- Navigate autonomously using gyroscope-based steering
- Detect and avoid obstacles using ultrasonic sensors
- Navigate through lanes with colored pillars (red and green)
- Execute precise turns and U-turns
- Complete racing tracks with timing measurements

The codebase is written in Python and interfaces with Raspberry Pi GPIO, I2C sensors, servo motors, and DC motors.

---

## Project Structure

```
GSGSmartiecarSG/
├── main/               # Main executable programs
│   ├── obstacle.py     # Obstacle avoidance challenge program
│   └── openrace.py     # Open race track challenge program
├── libs/               # Core library modules (reusable components)
│   ├── DCMotor.py      # DC motor control library
│   ├── gyro.py         # Gyroscope/IMU sensor interface
│   ├── steering.py     # Servo-based steering control
│   └── ultrasonic.py   # Ultrasonic distance sensor interface
└── Tools/              # Testing and diagnostic utilities
    ├── Led.py          # LED indicator control
    ├── Ultrasonic.py   # Ultrasonic sensor testing tool
    ├── gyro.py         # Gyroscope testing tool
    ├── i2c_test.py     # I2C bus scanner
    ├── motortest.py    # DC motor testing tool
    └── servotest.py    # Servo motor testing tool
```

---

## Main Programs

### 1. `main/obstacle.py`

**Purpose**: Competition program for obstacle avoidance challenge with colored pillars.

**Key Features**:
- Autonomous navigation through lanes with obstacles
- Color detection for red and green pillars
- Dynamic path planning based on pillar positions
- Gyroscope-based straight-line correction
- Corner counting and tracking
- Timed runs with performance measurement

**Main Flow**:
```
1. Initialize sensors and motors
2. Wait for button press to start
3. Determine driving direction (left/right)
4. Navigate lanes while:
   - Detecting pillars via camera (findNextPillar - not yet implemented)
   - Evading red pillars (pass on left side)
   - Evading green pillars (pass on right side)
   - Using ultrasonic sensors for wall following
5. Handle turns at lane ends
6. Stop at finish line
7. Display completion time
```

**Important Global Variables**:
- `DrivingDirection`: 'L' (left), 'R' (right), or 'U' (unknown)
- `Distance_Front/Left/Right`: Ultrasonic sensor readings in cm
- `StraightAngle`: Reference angle for straight driving
- `corners`: Number of corners completed
- `P_color`: Detected pillar color ('R', 'G', or 'U')
- `P_x`: X-coordinate of detected pillar in camera frame

**Speed Settings**:
- `NormalSpeed`: 110 PWM units
- `SlowSpeed`: 110 PWM units
- `CurveSpeed`: 90 PWM units
- `StartSpeed`: 90 PWM units

### 2. `main/openrace.py`

**Purpose**: Competition program for open race track (time trial without obstacles).

**Key Features**:
- Wall-following navigation
- Gyroscope-based steering correction
- Curve detection and execution
- Corner counting for lap completion
- Direction detection (clockwise vs counterclockwise)

**Main Flow**:
```
1. Initialize and calibrate sensors
2. Wait for start button
3. Determine driving direction
4. Navigate track:
   - Follow inner wall (left or right)
   - Detect curves when wall distance exceeds threshold
   - Execute 90-degree turns using gyroscope
   - Maintain straight lines between turns
5. Complete 12 corners (3 laps)
6. Stop and display time
```

**Important Functions**:
- `Curve_L()` / `Curve_R()`: Execute 90-degree turns
- `alignLeft()` / `alignRight()`: Wall-following control
- `Gyro_steer_straight()`: Maintain straight trajectory
- `operateNavigationThroughTurns()`: Main navigation loop

**Speed Settings**:
- `NormalSpeed`: 70 PWM units
- `SlowSpeed`: 50 PWM units
- `CurveSpeed`: 100 PWM units

---

## Library Modules

### 1. `libs/DCMotor.py`

**Purpose**: Controls the main drive motor using PWM and GPIO pins.

**Hardware Pins**:
- `GSM1` (GPIO 5): PWM speed control
- `in1` (GPIO 6): Direction control 1
- `in2` (GPIO 7): Direction control 2

**Functions**:

#### `runMotor(Speed)`
Drives the motor forward at specified speed.
```python
# Speed: 0-255 (PWM duty cycle)
# Minimum speed: 185 to overcome motor inertia
```

**Technical Details**:
- If speed < 185, briefly applies 185 then target speed (soft start)
- Uses PWM duty cycle: (Speed/255.0) * 100

#### `runMotor_R(Speed)`
Drives the motor backward (reverse) at specified speed.
- Same soft-start logic as forward
- Used for backing up and reverse turns

#### `stopMotor()`
Stops the motor with active braking.
```python
# Sequence:
# 1. Set both direction pins LOW (coast)
# 2. Apply reverse PWM at 185 (active brake)
# 3. Delay 50ms
# 4. Set all pins LOW (full stop)
```

**Critical Note**: The module requires GPIO to be initialized in BCM mode before use.

### 2. `libs/gyro.py`

**Purpose**: Interfaces with BNO08x 9-axis IMU sensor for orientation tracking.

**Hardware**: Adafruit BNO08x IMU via I2C bus

**Functions**:

#### `gyro_start()`
Initializes the I2C connection and enables the game rotation vector feature.

#### `quaternionToRoll(qw, qx, qy, qz)`
**Complex Function - Explanation for Jury**:

Converts quaternion rotation data to a roll angle in degrees.

**Mathematical Background**:
- Quaternions represent 3D rotations using 4 numbers (w, x, y, z)
- Roll is rotation around the forward axis (car tilting left/right)
- Formula: `atan2(2*(w*z + x*y), w² + x² - y² - z²)`
- Converts radians to degrees: `* (180/π)`

**Why This Matters**: The IMU outputs quaternions to avoid gimbal lock. This function extracts the single-axis rotation we need for steering.

#### `IMU_getAngle()`
**Complex Function - Explanation for Jury**:

Returns the cumulative angle change since start, handling angle wrapping.

**Algorithm**:
```python
1. Get current roll angle from quaternion
2. Calculate change from last reading
3. Handle wraparound:
   - If change > 180°, subtract 360° (crossed from 179° to -180°)
   - If change < -180°, add 360° (crossed from -180° to 179°)
4. Add to cumulative angle
5. Return negative value (coordinate system correction)
```

**Why This Is Complex**: Angles wrap around at ±180°. Driving in a circle, the angle goes: 170° → 180° → -170° → -180° → 170°. Without correction, this would register as huge backward jumps instead of continuous forward rotation.

**Example**:
```
Last angle: 175°, Current angle: -178°
Raw change: -353° (wrong!)
Corrected: -353° + 360° = +7° (correct!)
```

### 3. `libs/steering.py`

**Purpose**: Controls the steering servo motor for turning.

**Hardware**: 
- Servo connected to channel 7 on PCA9685 (16-channel servo driver)
- ServoKit library via I2C

**Configuration**:
- `centered`: 70° (straight ahead)
- `max_left`: 30° (maximum left turn)
- `max_right`: 110° (maximum right turn)

**Functions**:

#### `center()`
Sets steering to straight ahead position (70°).

#### `left(Angle)`
Turns steering left by specified angle from center.
```python
# Input: Angle offset from center (0-40)
# Actual position: 70 - Angle
# Clamped to minimum 30° (safety limit)
```

#### `right(Angle)`
Turns steering right by specified angle from center.
```python
# Input: Angle offset from center (0-40)
# Actual position: 70 + Angle
# Clamped to maximum 110° (safety limit)
```

**Safety Features**: All functions enforce min/max limits to prevent mechanical damage to steering mechanism.

### 4. `libs/ultrasonic.py`

**Purpose**: Measures distances using HC-SR04 ultrasonic sensors in three directions.

**Hardware Pins**:
- `trigechoPin_F` (GPIO 1): Front sensor (trigger/echo shared)
- `trigechoPin_L` (GPIO 2): Left sensor
- `trigechoPin_R` (GPIO 0): Right sensor

**Functions**:

#### `VELOCITY_TEMP(temp)`
**Complex Function - Explanation for Jury**:

Calculates speed of sound adjusted for temperature.

**Physics Background**:
- Speed of sound in air: 331.5 m/s at 0°C
- Increases by 0.6 m/s per degree Celsius
- Formula: `(331.5 + 0.6 * temperature) * 100 / 1000000`
- Result in cm/µs for distance calculation

**Why This Matters**: Sound travels faster in warm air. At 20°C vs 0°C, this is a 3.6% difference. For a 100cm measurement, that's 3.6cm error if not corrected!

#### `SpaceUltraSonicFront()` / `SpaceUltraSonicLeft()` / `SpaceUltraSonicRight()`
**Complex Function - Explanation for Jury**:

Measures distance using ultrasonic pulse timing.

**Algorithm**:
```python
1. Send 10µs trigger pulse (creates ultrasonic burst)
2. Switch pin to input mode
3. Measure echo pulse width (time for sound to return)
4. Calculate distance: pulseWidth * velocity / 2
   - Divide by 2 because sound travels to object and back
5. Validate measurement (2-350cm range)
6. If invalid, return last valid measurement (noise rejection)
7. Add 20ms delay before next measurement (sensor cooldown)
```

**Timing Diagram**:
```
Trigger:  _____|‾‾‾‾‾|_______________
         (10µs pulse)

Echo:     ______________|‾‾‾‾‾‾‾‾‾|_____
         (pulse width = distance)
```

**Error Handling**: 
- Values < 2cm or > 350cm are considered noise
- Previous valid reading is used instead
- Prevents sudden "0cm" readings from causing crashes

---

## Testing & Utility Tools

### 1. `Tools/Led.py`

**Purpose**: Controls RGB LED indicators for status display.

**Hardware**: RGB LED via servo channels (unusual but creative wiring):
- Channel 10: Red
- Channel 11: Green  
- Channel 9: Blue

**Functions**:
- `led_r()`: Red (error/stop state)
- `led_g()`: Green (success/go)
- `led_b()`: Blue (turning/processing)
- `led_y()`: Yellow (ready/waiting)
- `led_off()`: All off

**Usage in Competition**:
- Yellow: Waiting for start button
- Blue: Executing turn
- Red/Green: Approaching corresponding pillar color
- Off/White: Normal driving

### 2. `Tools/Ultrasonic.py`

**Purpose**: Standalone testing tool for ultrasonic sensors.

**Features**:
- Measures all three sensors continuously
- German language output
- Displays formatted distances
- Proper cleanup on CTRL+C

**Usage**:
```bash
python Tools/Ultrasonic.py
```

**Output Example**:
```
Entfernung links = 45.3 cm
                             Entfernung rechts = 52.1 cm
                                                                  Entfernung vorne = 120.5 cm
```

### 3. `Tools/gyro.py`

**Purpose**: Test and debug the gyroscope sensor.

**Features**:
- Displays instantaneous angle (±180°)
- Displays cumulative total angle (unlimited)
- German output with proper formatting
- Error handling for I2C issues

**Usage**:
```bash
python Tools/gyro.py
```

**Use Cases**:
- Verify sensor installation
- Check I2C connection
- Calibrate sensor reading direction
- Test sensor responsiveness

### 4. `Tools/i2c_test.py`

**Purpose**: Scan I2C bus for connected devices.

**Output**: List of detected I2C addresses (hex format)

**Why Important**: The gyroscope and servo driver both use I2C. This tool verifies:
- I2C bus is working
- Devices are detected at expected addresses
- No address conflicts

**Expected Devices**:
- 0x4A or 0x4B: BNO08x IMU
- 0x40 or 0x60: PCA9685 servo driver

### 5. `Tools/motortest.py`

**Purpose**: Test DC motor forward/reverse operation.

**Test Sequence**:
```
1. Forward for 3 seconds at 20% speed
2. Stop for 3 seconds
3. Reverse for 3 seconds at 20% speed
4. Stop and cleanup
```

**Hardware Setup**:
- GPIO 26: Direction control
- GPIO 13: Speed (PWM at 650Hz)

### 6. `Tools/servotest.py`

**Purpose**: Test steering servo range of motion.

**Test Sequence**:
```
Loop:
  1. Center (70°) - 2 seconds
  2. Left (30°) - 2 seconds
  3. Center (70°) - 2 seconds
  4. Right (110°) - 2 seconds
```

**Usage**: Verify servo is properly connected and mechanical linkage is not binding.

---

## Hardware Integration

### Raspberry Pi GPIO Pin Mapping

**DC Motor** (via L298N or similar H-bridge):
- GPIO 5: PWM enable
- GPIO 6: Direction input 1
- GPIO 7: Direction input 2

**Ultrasonic Sensors** (HC-SR04):
- GPIO 0: Right sensor (trigger/echo)
- GPIO 1: Front sensor (trigger/echo)
- GPIO 2: Left sensor (trigger/echo)
- Note: Using single pin for both trigger and echo (special wiring)

**Button**:
- GPIO 4 (obstacle.py) or GPIO 2 (openrace.py): Start button

**I2C Bus** (SDA/SCL pins):
- BNO08x IMU (gyroscope)
- PCA9685 servo driver (16 channels)

**Via PCA9685 Servo Driver**:
- Channel 7: Steering servo
- Channels 9, 10, 11: RGB LED (unusual but functional)

### Sensor Specifications

**BNO08x IMU**:
- 9-axis sensor (accelerometer, gyroscope, magnetometer)
- I2C communication
- Provides quaternion output (no gimbal lock)
- Game rotation vector mode (ignores magnetometer for stability)

**HC-SR04 Ultrasonic Sensors**:
- Range: 2-350 cm
- Accuracy: ±3mm
- Ultrasonic frequency: 40kHz
- Operating voltage: 5V

**Servo Motor**:
- Standard hobby servo (assumed)
- 0-180° range (limited to 30-110° in software)
- 50Hz PWM control

---

## Communication Patterns

### Module Dependencies

```
main/obstacle.py
    ├── libs/DCMotor (motor control)
    ├── libs/gyro (orientation)
    ├── libs/steering (turning)
    ├── libs/ultrasonic (distance sensing)
    ├── Tools/Led (status display)
    └── libs/cam (camera - not yet implemented)

main/openrace.py
    ├── libs/DCMotor
    ├── libs/gyro
    ├── libs/steering
    ├── libs/ultrasonic
    └── Tools/Led

All libs/
    └── RPi.GPIO (low-level GPIO access)

libs/gyro, libs/steering
    └── I2C bus (hardware interface)
```

### Data Flow

```
Sensors → Libraries → Main Program → Actuators

Ultrasonic Sensors → libs/ultrasonic → Distance variables → Steering logic → libs/steering → Servo
Gyroscope → libs/gyro → Angle data → Correction calculation → libs/steering → Servo
Main Program → libs/DCMotor → H-Bridge → DC Motor
Main Program → Tools/Led → Status → RGB LED
```

### Control Loop (Simplified)

```python
while not_finished:
    # Sense
    distance_front = ultrasonic.SpaceUltraSonicFront()
    distance_left = ultrasonic.SpaceUltraSonicLeft()
    distance_right = ultrasonic.SpaceUltraSonicRight()
    current_angle = gyro.IMU_getAngle()
    
    # Think
    if need_to_turn():
        calculate_turn_angle()
    else:
        steering_correction = calculate_straight_correction(current_angle)
    
    # Act
    steering.left(correction_amount)  # or right()
    DCMotor.runMotor(speed)
```

---

## Complex Functions Explained

### 1. Gyroscope-Based Straight Line Driving

**Function**: `Gyro_steer_straight()` (appears in both main programs)

**Challenge**: Car naturally drifts due to motor variations, surface friction, wheel alignment.

**Solution**:
```python
def Gyro_steer_straight():
    angle = GY.IMU_getAngle()
    Steering = int((angle - StraightAngle) * 0.8)  # Proportional control
    
    # Clamp steering to prevent over-correction
    if Steering > 15.0:
        Steering = 15
    elif Steering < -15.0:
        Steering = -15
    
    # Apply correction
    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)
```

**How It Works**:
1. Calculate angle error: `error = current_angle - target_angle`
2. Apply proportional gain: `correction = error * 0.8`
   - 0.8 is the "wiggle factor" (reduces oscillation)
3. Clamp output to ±15° (prevents violent steering)
4. Apply correction in correct direction

**Why 0.8 Factor?**: 
- If gain = 1.0, system oscillates (over-corrects)
- If gain = 0.5, system responds too slowly
- 0.8 is experimentally tuned for smooth tracking

**PID Note**: This is a P-controller (proportional only). No I (integral) or D (derivative) terms. Simple but effective for this application.

### 2. Wall Following with Ultrasonic Sensors

**Function**: `alignLeft()` / `alignRight()` (openrace.py)

**Challenge**: Follow a wall at constant distance while moving forward.

**Solution**:
```python
def alignLeft():
    global Distance_Left
    Distance_Left = US.SpaceUltraSonicLeft()
    Steering = int((Distance_Left - Walldistance) * 0.9)
    
    # Clamp
    if Steering > 30.0:
        Steering = 30
    elif Steering < -30.0:
        Steering = -30
    
    # Apply
    if Steering < 0:
        Steering = Steering * (-1)
        ST.right(Steering)
    else:
        ST.left(Steering)
    
    delay(20)
```

**How It Works**:
1. Measure distance to wall
2. Calculate error: `error = measured - target`
3. Convert to steering: `steering = error * 0.9`
   - Positive error (too far): steer left
   - Negative error (too close): steer right
4. Clamp to mechanical limits
5. Update every 20ms

**Target Distance**: `Walldistance = 30cm` (openrace.py)

**Why 0.9 Factor?**: Higher gain than straight-line (0.8) because wall following requires more responsive corrections.

### 3. Turn Execution with Angle Feedback

**Function**: `Curve_L()` / `Curve_R()` (openrace.py)

**Challenge**: Execute precise 90° turns while maintaining speed.

**Complex Algorithm**:
```python
def Curve_R():
    global corners, StraightAngle, angle, LastCurveTime, Distance_Right
    
    Speed = CurveSpeed  # Start speed
    ST.right(40)  # Apply full steering
    
    TargetDirection = StraightAngle + 90.0  # Calculate target
    angle = GY.IMU_getAngle()
    StraightAngle = TargetDirection  # Update reference
    
    DC.runMotor(SlowSpeed)
    
    # Turn until reaching target, with speed ramp
    while angle < TargetDirection - correction_Right:
        angle = GY.IMU_getAngle()
        Speed = Speed + 5  # Accelerate during turn
        if Speed > CurveSpeed:
            Speed = CurveSpeed  # Cap at max curve speed
        DC.runMotor(Speed)
        delay(50)
    
    corners = corners + 1
    ST.center()
    DC.runMotor(NormalSpeed)
    
    # Wait to detect inner wall again
    Distance_Right = US.SpaceUltraSonicRight()
    while Distance_Right > 60:
        Distance_Right = US.SpaceUltraSonicRight()
        Gyro_steer_straight()
    
    LastCurveTime = millis()  # Record turn time
```

**Why This Is Complex**:

1. **Speed Ramping**: Starts slow, accelerates during turn for smooth cornering
2. **Angle Feedback**: Uses gyroscope to measure actual rotation
3. **Correction Factor**: `correction_Right` (5.0) accounts for turn overshoot
4. **Wall Reacquisition**: After turn, searches for inner wall to resume wall following
5. **Time Gating**: Records turn time to prevent detecting same turn twice

**Coordinate System**:
- Right turn: angle increases (0° → 90° → 180° → 270°)
- Left turn: angle decreases (0° → -90° → -180° → -270°)

### 4. Pillar Evasion Maneuvers

**Function**: `EvadeRedPillar()` / `EvadeGreenPillar()` (obstacle.py)

**Challenge**: Detect colored pillar, steer around it, and return to straight path.

**Algorithm for Red Pillar (evade on left)**:
```python
def EvadeRedPillar():
    global angle, StraightAngle, P_x, P_color, LastPillarColor
    
    L.led_r()  # Show red LED
    angle = GY.IMU_getAngle()
    TargetDirection = StraightAngle  # Remember straight angle
    
    DC.runMotor(SlowSpeed)
    ST.right(35)  # Steer right to go around
    
    # findNextPillar() - camera function not yet implemented
    
    # Turn until pillar is on the right side (P_x > 30)
    while (P_x > 30) and (P_color == 'R'):
        # findNextPillar()
        pass
    
    ST.center()
    delay(200)  # Coast past pillar
    
    # Wait until pillar is out of view
    while P_color == 'R':
        # findNextPillar()
        delay(200)
    
    # Return to straight
    ST.left(35)
    angle = GY.IMU_getAngle()
    while angle > TargetDirection:
        angle = GY.IMU_getAngle()
        delay(10)
    
    ST.center()
    DC.runMotor(SlowSpeed)
    
    # Drive completely past pillar
    prev_x = P_x
    # findNextPillar()
    while (P_color == 'R') and (P_x <= prev_x):
        P_x = prev_x
        # findNextPillar()
        Gyro_steer_straight()
    
    delay(100)
    L.led_off()
    LastPillarColor = 'R'
```

**Phases**:
1. **Approach**: Pillar detected ahead (via camera - not yet implemented)
2. **Initial Steer**: Turn away from pillar
3. **Wait for Clear View**: Pillar moves to side (changes X position)
4. **Coast**: Go straight briefly
5. **Wait for Pass**: Continue until pillar leaves camera view
6. **Return**: Counter-steer back to original heading
7. **Verify**: Check pillar X position decreasing (moving behind)

**Why Complex**:
- Combines vision (camera), orientation (gyro), and motion (motor/steering)
- State machine with multiple phases
- Must handle partial views and occlusion
- Returns to exact original heading using gyroscope feedback

**Green Pillar**: Mirror image - evades on right side instead of left.

### 5. U-Turn Execution

**Function**: `UTurnRight()` / `UTurnLeft()` (obstacle.py)

**Challenge**: Perform 180° turn in confined space.

**Algorithm (Right U-Turn)**:
```python
def UTurnRight():
    TargetDirection = 0.0
    angle = 0.0
    
    # Phase 1: Forward turn 45°
    DC.runMotor(SlowSpeed)
    TargetDirection = StraightAngle + 45.0
    ST.right(45)
    angle = GY.IMU_getAngle()
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()
    
    # Phase 2: Reverse turn 45° (total 90°)
    ST.left(45)
    TargetDirection = StraightAngle + 90.0
    DC.runMotor_R(SlowSpeed)
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()
    
    # Phase 3: Forward turn 45° (total 135°)
    TargetDirection = StraightAngle + 135.0
    ST.right(45)
    angle = GY.IMU_getAngle()
    DC.runMotor(SlowSpeed)
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()
    
    # Phase 4: Reverse turn 45° (total 180°)
    ST.left(45)
    TargetDirection = StraightAngle + 180.0
    DC.runMotor_R(SlowSpeed)
    while angle < TargetDirection:
        angle = GY.IMU_getAngle()
    DC.stopMotor()
    
    ST.center()
    DC.runMotor(SlowSpeed)
    
    return TargetDirection
```

**Why 4 Phases?**:
- Forward-only or reverse-only U-turn requires large turning radius
- Alternating forward/reverse = tighter turn (like parallel parking)
- 45° increments keep car within lane width
- Each phase uses opposite steering direction

**Visualization**:
```
Start: →
Phase 1: Forward right ↗
Phase 2: Reverse left ↖
Phase 3: Forward right ↖ (continuing turn)
Phase 4: Reverse left ←
End: ← (opposite direction)
```

### 6. Direction Detection

**Function**: `slowspeedToFindCorrectDirection()` (openrace.py)

**Challenge**: Determine if track runs clockwise or counterclockwise.

**Algorithm**:
```python
def slowspeedToFindCorrectDirection():
    global DrivingDirection, Distance_Left, Distance_Right
    
    if DrivingDirection == 'U':  # Unknown
        DC.runMotor(SlowSpeed)
        # Drive forward until one side opens up
        while (Distance_Left < 80.0) and (Distance_Right < 80.0):
            Distance_Left = US.SpaceUltraSonicLeft()
            Distance_Right = US.SpaceUltraSonicRight()
            Gyro_steer_straight()
        
        # First opening determines direction
        if Distance_Left >= 80.0:
            DrivingDirection = 'L'
            Curve_L()
        elif Distance_Right >= 80.0:
            DrivingDirection = 'R'
            Curve_R()
```

**Why Needed**: 
- Start position may be on either side of track
- Must follow inner wall (shorter path)
- Once detected, direction stays constant for entire run

**Detection Threshold**: 80cm (indicates open turn vs. straight wall)

---

## Setup and Dependencies

### Required Python Libraries

```bash
# System libraries (installed via apt)
sudo apt-get install python3-pip python3-dev

# Python packages (install via pip)
pip3 install RPi.GPIO
pip3 install adafruit-circuitpython-bno08x
pip3 install adafruit-circuitpython-servokit
pip3 install adafruit-circuitpython-motor
```

### Hardware Requirements

**Minimum**:
- Raspberry Pi 3B+ or newer
- DC motor with H-bridge driver (L298N or similar)
- Servo motor for steering
- 3× HC-SR04 ultrasonic sensors
- BNO08x IMU (I2C)
- PCA9685 16-channel servo driver (I2C)
- RGB LED
- Push button (start)
- 7.4V LiPo battery (2S) for motors
- 5V power for Raspberry Pi

**Optional**:
- Camera module (for pillar detection in obstacle.py)
- LCD display (referenced but not implemented)

### Wiring Diagram Summary

```
Raspberry Pi GPIO:
├── 0, 1, 2: Ultrasonic sensors
├── 4 or 2: Start button
├── 5, 6, 7: DC motor H-bridge
└── SDA/SCL (I2C):
    ├── BNO08x IMU (0x4A/0x4B)
    └── PCA9685 Servo Driver (0x40/0x60)
        ├── Channel 7: Steering servo
        └── Channels 9,10,11: RGB LED
```

### Initial Setup Steps

1. **Enable I2C**:
```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

2. **Test I2C Connection**:
```bash
sudo i2cdetect -y 1
# Should show devices at 0x4A (IMU) and 0x40 (servo driver)
```

3. **Test Components**:
```bash
# Test I2C
python3 Tools/i2c_test.py

# Test gyroscope
python3 Tools/gyro.py

# Test ultrasonic sensors
python3 Tools/Ultrasonic.py

# Test motor
python3 Tools/motortest.py

# Test steering
python3 Tools/servotest.py

# Test LED
python3 Tools/Led.py
```

4. **Calibrate Sensors**:
- **Gyro**: Place car on level surface, run `Tools/gyro.py`, verify angle starts at ~0°
- **Ultrasonic**: Point at wall 30cm away, verify readings are consistent
- **Steering**: Adjust `centered` value in `steering.py` if car doesn't track straight

5. **Run Main Programs**:
```bash
# Open race (time trial)
python3 main/openrace.py

# Obstacle course
python3 main/obstacle.py
```

### Configuration Parameters

**openrace.py**:
- `SlowSpeed = 50`: Starting/turning speed
- `NormalSpeed = 70`: Straight-line speed
- `CurveSpeed = 100`: Maximum turn speed
- `Walldistance = 30`: Target wall following distance (cm)
- `NextCurveDelay = 1000`: Minimum ms between detecting turns

**obstacle.py**:
- `NormalSpeed = 110`: Straight-line speed
- `SlowSpeed = 110`: Approach/maneuver speed
- `CurveSpeed = 90`: Turn speed
- `walldistance = 40`: Target wall distance (cm)
- `NextCurveDelay = 5000`: Minimum ms between turns

**Tuning Tips**:
- Increase speeds on smooth surfaces
- Decrease speeds if car overshoots turns
- Adjust wall distance based on track width
- Increase delays if car detects same turn twice

### Common Issues and Solutions

**Issue**: Car drifts to one side when going "straight"
- **Solution**: Adjust `centered` in `steering.py` (try ±5°)

**Issue**: Ultrasonic sensors return 0 or 350
- **Solution**: Check wiring, verify 5V power, add delay between readings

**Issue**: Gyroscope readings drift over time
- **Solution**: Normal for IMU without magnetometer, use short runs or periodic reset

**Issue**: Car over-corrects and oscillates
- **Solution**: Reduce gain factors (0.8 → 0.6) in `Gyro_steer_straight()`

**Issue**: Motor doesn't start (just hums)
- **Solution**: Increase minimum speed threshold (185 → 200) in `DCMotor.py`

**Issue**: I2C devices not detected
- **Solution**: 
  1. Run `sudo i2cdetect -y 1`
  2. Check 3.3V power and ground
  3. Verify SDA/SCL connections
  4. Try different I2C address

---

## Competition Strategy Notes

### Open Race Strategy
1. **Start**: Determine direction on first straight
2. **Follow Wall**: Maintain constant distance from inner wall
3. **Turn Detection**: Monitor sensor for >80cm reading
4. **Turn Execution**: Full steering + speed ramp for smooth corner
5. **Wall Reacquisition**: Search for wall after turn
6. **Lap Counting**: Stop after 12 corners (3 laps of 4-corner track)

**Key Performance Factors**:
- Higher `NormalSpeed` = faster lap time
- Higher `CurveSpeed` = faster through corners
- Lower `correction` factors = sharper but riskier turns

### Obstacle Course Strategy
1. **Start Phase**: Detect first pillar and initial direction
2. **Lane Navigation**: Follow lane while detecting pillars
3. **Pillar Evasion**: Red left, green right (pass on opposite side)
4. **Multiple Pillars**: Check for second pillar after evading first
5. **Turn Handling**: Gyro-based turns at lane ends
6. **Finish**: Drive to end wall and stop precisely

**Key Performance Factors**:
- Camera reliability (currently not implemented)
- Pillar detection distance (earlier = smoother evasion)
- Evasion angle (35° is balance of clearance vs. time)
- Turn precision (affects alignment for next lane)

---

## Future Improvements

### Currently Unimplemented

1. **Camera Module** (`libs/cam`):
   - Referenced in obstacle.py but not implemented
   - Would enable `findNextPillar()` function
   - Critical for pillar color detection

2. **LCD Display**:
   - Code contains commented `lcd.setCursor()` calls
   - Would show real-time status during runs

3. **Advanced PID Control**:
   - Current system uses P-only control
   - Adding I and D terms would improve:
     - Steady-state error (I term)
     - Dampening/stability (D term)

### Suggested Enhancements

1. **Kalman Filter**:
   - Fuse gyroscope + ultrasonic data for better position estimate
   - Reduce sensor noise effects

2. **Path Planning**:
   - Pre-calculate optimal trajectory around pillars
   - Minimize path length

3. **Speed Optimization**:
   - Dynamic speed adjustment based on upcoming features
   - Slow before turns, accelerate on straights

4. **Error Recovery**:
   - Detect and recover from off-track situations
   - Wall collision detection and backup

5. **Telemetry**:
   - Log sensor data and decisions during runs
   - Post-run analysis for tuning

6. **State Machine**:
   - Formal state machine for obstacle course
   - Clearer logic flow and easier debugging

---

## Glossary

**Terms for Jury/Non-Technical Reviewers**:

- **GPIO**: General Purpose Input/Output - programmable pins on Raspberry Pi
- **PWM**: Pulse Width Modulation - technique to control motor speed by varying power
- **I2C**: Inter-Integrated Circuit - communication protocol for connecting sensors
- **IMU**: Inertial Measurement Unit - sensor that measures orientation and acceleration
- **Quaternion**: Mathematical representation of 3D rotation (4 numbers instead of 3 angles)
- **Servo**: Motor that moves to specific angles (used for steering)
- **DC Motor**: Direct Current motor for propulsion (speed controlled by voltage)
- **Ultrasonic**: Sound above human hearing range (used for distance measurement)
- **H-Bridge**: Circuit that allows DC motor to run forward and reverse
- **PID**: Proportional-Integral-Derivative - control algorithm for smooth tracking
- **Roll**: Rotation around forward axis (tilting left/right)
- **Pitch**: Rotation around left-right axis (nose up/down)
- **Yaw**: Rotation around vertical axis (turning left/right)

---

## Contact and Support

**Original Project**: https://github.com/Overdriven187/GSG_SmartiecarV2

**Code Structure**:
- Python 3.7+
- Raspberry Pi OS (Debian-based)
- Real-time control with hardware interfacing

**Authors**: GSG Robotics Team
**Competition**: Future Engineers / WRO (World Robot Olympiad)

---

**Last Updated**: October 2025
**Documentation Version**: 1.0
**Code Status**: Active Development - Camera module pending implementation
