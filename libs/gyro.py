import time
import board
import busio
import math
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

bno = None
i2c = None
CompAngle = 0.0
LastAngle = 0.0

def gyro_start():
    global i2c
    global bno
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
    print("gyro start")


def quaternionToRoll(qw, qx, qy, qz):
    roll = math.atan2(2.0 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)
    return roll * (180.0 / math.pi)

def IMU_getAngle():
    global CompAngle
    global LastAngle
    global bno
    quat_i, quat_j, quat_k, quat_real = bno.game_quaternion  # pylint:disable=no-member
    angle = quaternionToRoll(quat_real, quat_i, quat_j, quat_k)
    change = angle - LastAngle
    if change > 180.0:
        change = change - 360.0
        
    if change < -180.0:
        change = change + 360.0
        
    CompAngle = CompAngle + change
    LastAngle = angle 

    return CompAngle*(-1)



if __name__ == '__main__':
    try:
        while True:
            time.sleep(0.5)           
            comp = IMU_getAngle()
           
            print("comp: ",comp)
    except:
        pass