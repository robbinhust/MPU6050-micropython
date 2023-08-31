import ustruct
from machine import I2C, Pin


MPU6050_ADDR            = 0x69
MPU6050_CONFIG          = 0x1a
MPU6050_GYRO_CONFIG     = 0x1b
MPU6050_ACCEL_CONFIG    = 0x1c
MPU6050_INT_PIN_CFG     = 0x37
MPU6050_INT_ENABLE      = 0x38
MPU6050_INT_STATUS      = 0x3a
MPU6050_ACCEL_XOUT_H    = 0x3b
MPU6050_ACCEL_XOUT_L    = 0x3c
MPU6050_ACCEL_YOUT_H    = 0x3d
MPU6050_ACCEL_YOUT_L    = 0x3e
MPU6050_ACCEL_ZOUT_H    = 0x3f
MPU6050_ACCEL_ZOUT_L    = 0x40
MPU6050_TEMP_OUT_H      = 0x41
MPU6050_TEMP_OUT_L      = 0x42
MPU6050_GYRO_XOUT_H     = 0x43
MPU6050_GYRO_XOUT_L     = 0x44
MPU6050_GYRO_YOUT_H     = 0x45
MPU6050_GYRO_YOUT_L     = 0x46
MPU6050_GYRO_ZOUT_H     = 0x47
MPU6050_GYRO_ZOUT_L     = 0x48
MPU6050_PWR_MGMT_1      = 0x6b
MPU6050_SMPLRT_DIV      = 0x19

ACCEL_FS_SEL_2G         = 0b00000000
ACCEL_FS_SEL_4G         = 0b00001000
ACCEL_FS_SEL_8G         = 0b00010000
ACCEL_FS_SEL_16G        = 0b00011000

_ACCEL_SO_2G = 16384 # 1 / 16384 ie. 0.061 mg / digit
_ACCEL_SO_4G = 8192 # 1 / 8192 ie. 0.122 mg / digit
_ACCEL_SO_8G = 4096 # 1 / 4096 ie. 0.244 mg / digit
_ACCEL_SO_16G = 2048 # 1 / 2048 ie. 0.488 mg / digit

GYRO_FS_SEL_250DPS      = 0b00000000
GYRO_FS_SEL_500DPS      = 0b00001000
GYRO_FS_SEL_1000DPS     = 0b00010000
GYRO_FS_SEL_2000DPS     = 0b00011000

_GYRO_SO_250DPS         = 131
_GYRO_SO_500DPS         = 65.5
_GYRO_SO_1000DPS        = 32.8
_GYRO_SO_2000DPS        = 16.4

SF_G        = 1
SF_M_S2     = 9.80665 # 1 g = 9.80665 m/s2 ie. standard gravity
SF_DEG_S    = 1
SF_RAD_S    = 0.017453292519943 # 1 deg/s is 0.017453292519943 rad/s

class MPU6050():
    def __init__(self, i2c = None, accel_fs = ACCEL_FS_SEL_2G, gyro_fs = GYRO_FS_SEL_250DPS, accel_sf = SF_M_S2, gyro_sf = SF_RAD_S, gyro_offset = (0,0,0)):
        self.accel_fs = accel_fs
        self.gyro_fs = gyro_fs
        self.accel_sf = accel_sf
        self.gyro_sf = gyro_sf
        if not i2c:
            self.i2c = i2c = I2C(scl=Pin(22), sda=Pin(21), freq=100000)
        else:
            self.i2c = i2c
        self.addr = MPU6050_ADDR
        self._gyro_offset = gyro_offset
        self.i2c.writeto_mem(MPU6050_ADDR, MPU6050_PWR_MGMT_1, bytes([0x00])) # SLEEP
        self.i2c.writeto_mem(MPU6050_ADDR, MPU6050_INT_ENABLE, bytes([0x01])) # DATA_RDY_EN
        self._accel_so = self.accel_config(accel_fs)
        self._gyro_so = self.gyro_config(gyro_fs)


    def accel_config(self, value):
        self.i2c.writeto_mem(self.addr, MPU6050_ACCEL_CONFIG, bytearray([value]))
        if ACCEL_FS_SEL_2G == value:
            return _ACCEL_SO_2G
        elif ACCEL_FS_SEL_4G == value:
            return _ACCEL_SO_4G
        elif ACCEL_FS_SEL_8G == value:
            return _ACCEL_SO_8G
        elif ACCEL_FS_SEL_16G == value:
            return _ACCEL_SO_16G
        
    def gyro_config(self, value):
        self.i2c.writeto_mem(self.addr, MPU6050_GYRO_CONFIG, bytearray([value]))
        if GYRO_FS_SEL_250DPS == value:
            return _GYRO_SO_250DPS
        elif GYRO_FS_SEL_500DPS == value:
            return _GYRO_SO_500DPS
        elif GYRO_FS_SEL_1000DPS == value:
            return _GYRO_SO_1000DPS
        elif GYRO_FS_SEL_2000DPS == value:
            return _GYRO_SO_2000DPS
    
    @property
    def acceleration(self):
        """
        Acceleration measured by the sensor. By default will return a
        3-tuple of X, Y, Z axis acceleration values in m/s^2 as floats. Will
        return values in g if constructor was provided `accel_sf=SF_M_S2`
        parameter.
        """
        so = self._accel_so
        sf = self.accel_sf

        xyz = self._register_three_shorts(MPU6050_ACCEL_XOUT_H)
        return tuple([value / so * sf for value in xyz])

    @property
    def gyro(self):
        """
        X, Y, Z radians per second as floats.
        """
        so = self._gyro_so
        sf = self.gyro_sf
        ox, oy, oz = self._gyro_offset

        xyz = self._register_three_shorts(MPU6050_GYRO_XOUT_H)
        xyz = [value / so * sf for value in xyz]

        xyz[0] -= ox
        xyz[1] -= oy
        xyz[2] -= oz

        return tuple(xyz)
    
    def _register_short(self, register, value=None, buf=bytearray(2)):
        if value is None:
            self.i2c.readfrom_mem_into(self.addr, register, buf)
            return ustruct.unpack(">h", buf)[0]

        ustruct.pack_into(">h", buf, 0, value)
        return self.i2c.writeto_mem(self.addr, register, buf)
    
    def _register_three_shorts(self, register, buf=bytearray(6)):
        self.i2c.readfrom_mem_into(self.addr, register, buf)
        return ustruct.unpack(">hhh", buf)
