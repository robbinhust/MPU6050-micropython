# MicroPython MPU-6050 I2C driver

## Usage

```python
from mpu6050 import MPU6050
from machine import *

i2c = I2C(scl=Pin(22), sda=Pin(21), freq=100000)
mpu6050 = MPU6050(i2c)

while True:
    ax, ay, az = mpu6050.acceleration
    gx, gy, gz = mpu6050.gyro
    print(ax, ay, az, gx, gy, gz)
    sleep_ms(100)
```

By default the library returns 3-tuple of X, Y, Z axis values for either acceleration, gyroscope. Default units are `m/s^2`, `rad/s`. It is possible to also get acceleration values in `g` and gyro values `deg/s`. See the example below.

```python
from machine import *
from mpu6050 import MPU6050, SF_G, SF_DEG_S

i2c = I2C(scl=Pin(22), sda=Pin(21), freq=100000)
mpu6050 = MPU6500(i2c, accel_sf=SF_G, gyro_sf=SF_DEG_S)

while True:
    ax, ay, az = mpu6050.acceleration
    gx, gy, gz = mpu6050.gyro
    print(ax, ay, az, gx, gy, gz)
    sleep_ms(100)
```

Code stolen from [tuupola's mpu6500](https://github.com/tuupola/micropython-mpu9250/blob/master/mpu6500.py)
