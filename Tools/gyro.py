# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
import math
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

bno = None
i2c = None
totalangle = 0.0
lastangle = 0.0

def gyro_start():
    global i2c
    global bno
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
    print("gyro startet")


def quaternionToRoll(qw, qx, qy, qz):
    roll = math.atan2(2.0 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)
# Drehung nach links positiv nach rechts negativ
    return roll * (180.0 / math.pi)

def CalcAngle():
    global totalangle
    global lastangle
    global bno

    # Try to read the quaternion data, with error handling for I2C issues
    try:
        quat_i, quat_j, quat_k, quat_real = bno.game_quaternion  # pylint:disable=no-member
    except KeyError as e:
        print(f"Error processing quaternion data: {e}")
        return 0.0, totalangle
    except OSError as e:
        print(f"I2C communication error: {e}")
        # Optional: Retry after a small delay
        time.sleep(0.1)  # Short delay before retry
        return CalcAngle()  # Retry the function

    angle = quaternionToRoll(quat_real, quat_i, quat_j, quat_k)
    change = angle - lastangle
    if change > 180.0:
        change = change - 360.0
        
    if change < -180.0:
        change = change + 360.0
        
    totalangle = totalangle + change
    lastangle = angle

    return angle * (-1), totalangle * (-1)



if __name__ == '__main__':
    try:
        gyro_start()
        while True:
            time.sleep(0.2)

            angle, total = CalcAngle()
           
            print("winkel: ", angle)
            print("gesamt: ", total)
           
            
    except KeyboardInterrupt:
        print("Messung vom User gestoppt")
        GPIO.cleanup()
 