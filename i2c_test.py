import busio
import board
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA)
print("Scanning for I2C devices...")
while not i2c.try_lock():
    pass
devices = i2c.scan()
print("Found devices:", devices)
i2c.unlock()
