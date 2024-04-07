import time
import struct
import math
import numpy as np

magnet_calibrate_list = [-70,220] #!地磁気補正データ

__version__ = '0.0.2'

CHIP_ID = 0xEA
I2C_ADDR = 0x68
I2C_ADDR_ALT = 0x69
ICM20948_BANK_SEL = 0x7f

ICM20948_I2C_MST_ODR_CONFIG = 0x00
ICM20948_I2C_MST_CTRL = 0x01
ICM20948_I2C_MST_DELAY_CTRL = 0x02
ICM20948_I2C_SLV0_ADDR = 0x03
ICM20948_I2C_SLV0_REG = 0x04
ICM20948_I2C_SLV0_CTRL = 0x05
ICM20948_I2C_SLV0_DO = 0x06
ICM20948_EXT_SLV_SENS_DATA_00 = 0x3B

ICM20948_GYRO_SMPLRT_DIV = 0x00
ICM20948_GYRO_CONFIG_1 = 0x01
ICM20948_GYRO_CONFIG_2 = 0x02

# Bank 0
ICM20948_WHO_AM_I = 0x00
ICM20948_USER_CTRL = 0x03
ICM20948_PWR_MGMT_1 = 0x06
ICM20948_PWR_MGMT_2 = 0x07
ICM20948_INT_PIN_CFG = 0x0F

ICM20948_ACCEL_SMPLRT_DIV_1 = 0x10
ICM20948_ACCEL_SMPLRT_DIV_2 = 0x11
ICM20948_ACCEL_INTEL_CTRL = 0x12
ICM20948_ACCEL_WOM_THR = 0x13
ICM20948_ACCEL_CONFIG = 0x14
ICM20948_ACCEL_XOUT_H = 0x2D
ICM20948_GRYO_XOUT_H = 0x33

ICM20948_TEMP_OUT_H = 0x39
ICM20948_TEMP_OUT_L = 0x3A

# Offset and sensitivity - defined in electrical characteristics, and TEMP_OUT_H/L of datasheet
ICM20948_TEMPERATURE_DEGREES_OFFSET = 21
ICM20948_TEMPERATURE_SENSITIVITY = 333.87
ICM20948_ROOM_TEMP_OFFSET = 21

AK09916_I2C_ADDR = 0x0c
AK09916_CHIP_ID = 0x09
AK09916_WIA = 0x01
AK09916_ST1 = 0x10
AK09916_ST1_DOR = 0b00000010   # Data overflow bit
AK09916_ST1_DRDY = 0b00000001  # Data self.ready bit
AK09916_HXL = 0x11
AK09916_ST2 = 0x18
AK09916_ST2_HOFL = 0b00001000  # Magnetic sensor overflow bit
AK09916_CNTL2 = 0x31
AK09916_CNTL2_MODE = 0b00001111
AK09916_CNTL2_MODE_OFF = 0
AK09916_CNTL2_MODE_SINGLE = 1
AK09916_CNTL2_MODE_CONT1 = 2
AK09916_CNTL2_MODE_CONT2 = 4
AK09916_CNTL2_MODE_CONT3 = 6
AK09916_CNTL2_MODE_CONT4 = 8
AK09916_CNTL2_MODE_TEST = 16
AK09916_CNTL3 = 0x32


class ICM20948:
    def write(self, reg, value):
        """Write byte to the sensor."""
        self._bus.write_byte_data(self._addr, reg, value)
        time.sleep(0.0001)

    def read(self, reg):
        """Read byte from the sensor."""
        return self._bus.read_byte_data(self._addr, reg)

    def trigger_mag_io(self):
        user = self.read(ICM20948_USER_CTRL)
        self.write(ICM20948_USER_CTRL, user | 0x20)
        time.sleep(0.005)
        self.write(ICM20948_USER_CTRL, user)

    def read_bytes(self, reg, length=1):
        """Read byte(s) from the sensor."""
        return self._bus.read_i2c_block_data(self._addr, reg, length)

    def bank(self, value):
        """Switch register self.bank."""
        if not self._bank == value:
            self.write(ICM20948_BANK_SEL, value << 4)
            self._bank = value

    def mag_write(self, reg, value):
        """Write a byte to the slave magnetometer."""
        self.bank(3)
        self.write(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR)  # Write one byte
        self.write(ICM20948_I2C_SLV0_REG, reg)
        self.write(ICM20948_I2C_SLV0_DO, value)
        self.bank(0)
        self.trigger_mag_io()

    def mag_read(self, reg):
        """Read a byte from the slave magnetometer."""
        self.bank(3)
        self.write(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80)
        self.write(ICM20948_I2C_SLV0_REG, reg)
        self.write(ICM20948_I2C_SLV0_DO, 0xff)
        self.write(ICM20948_I2C_SLV0_CTRL, 0x80 | 1)  # Read 1 byte

        self.bank(0)
        self.trigger_mag_io()

        return self.read(ICM20948_EXT_SLV_SENS_DATA_00)

    def mag_read_bytes(self, reg, length=1):
        """Read up to 24 bytes from the slave magnetometer."""
        self.bank(3)
        self.write(ICM20948_I2C_SLV0_CTRL, 0x80 | 0x08 | length)
        self.write(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80)
        self.write(ICM20948_I2C_SLV0_REG, reg)
        self.write(ICM20948_I2C_SLV0_DO, 0xff)
        self.bank(0)
        self.trigger_mag_io()

        return self.read_bytes(ICM20948_EXT_SLV_SENS_DATA_00, length)

    def magnetometer_ready(self):
        """Check the magnetometer status self.ready bit."""
        return self.mag_read(AK09916_ST1) & 0x01 > 0

    def read_magnetometer_data(self, timeout=1.0):
        self.mag_write(AK09916_CNTL2, 0x01)  # Trigger single measurement
        t_start = time.time()
        while not self.magnetometer_ready():
            if time.time() - t_start > timeout:
                raise RuntimeError("Timeout waiting for Magnetometer Ready")
            time.sleep(0.00001)

        data = self.mag_read_bytes(AK09916_HXL, 6)

        # Read ST2 to confirm self.read finished,
        # needed for continuous modes
        # self.mag_read(AK09916_ST2)

        x, y, z = struct.unpack("<hhh", bytearray(data))

        # Scale for magnetic flux density "uT"
        # from section 3.3 of the datasheet
        # This value is constant
        x *= 0.15
        y *= 0.15
        z *= 0.15

        return x, y, z

    def read_accelerometer_gyro_data(self):
        self.bank(0)
        data = self.read_bytes(ICM20948_ACCEL_XOUT_H, 12)

        ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhh", bytearray(data))

        self.bank(2)

        # Read accelerometer full scale range and
        # use it to compensate the self.reading to gs
        scale = (self.read(ICM20948_ACCEL_CONFIG) & 0x06) >> 1

        # scale ranges from section 3.2 of the datasheet
        gs = [16384.0, 8192.0, 4096.0, 2048.0][scale]

        ax /= gs
        ay /= gs
        az /= gs

        # Read back the degrees per second rate and
        # use it to compensate the self.reading to dps
        scale = (self.read(ICM20948_GYRO_CONFIG_1) & 0x06) >> 1

        # scale ranges from section 3.1 of the datasheet
        dps = [131, 65.5, 32.8, 16.4][scale]

        gx /= dps
        gy /= dps
        gz /= dps

        return ax, ay, az, gx, gy, gz
    
    def read_accelerometer_data(self):
        self.bank(0)
        data = self.read_bytes(ICM20948_ACCEL_XOUT_H, 12)

        ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhh", bytearray(data))

        self.bank(2)

        # Read accelerometer full scale range and
        # use it to compensate the self.reading to gs
        scale = (self.read(ICM20948_ACCEL_CONFIG) & 0x06) >> 1

        # scale ranges from section 3.2 of the datasheet
        gs = [16384.0, 8192.0, 4096.0, 2048.0][scale]

        ax /= gs
        ay /= gs
        az /= gs

        # Read back the degrees per second rate and
        # use it to compensate the self.reading to dps
        scale = (self.read(ICM20948_GYRO_CONFIG_1) & 0x06) >> 1

        # scale ranges from section 3.1 of the datasheet
        dps = [131, 65.5, 32.8, 16.4][scale]

        gx /= dps
        gy /= dps
        gz /= dps

        return ax, ay, az
    
    def read_gyro_data(self):
        self.bank(0)
        data = self.read_bytes(ICM20948_ACCEL_XOUT_H, 12)

        ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhh", bytearray(data))

        self.bank(2)

        # Read accelerometer full scale range and
        # use it to compensate the self.reading to gs
        scale = (self.read(ICM20948_ACCEL_CONFIG) & 0x06) >> 1

        # scale ranges from section 3.2 of the datasheet
        gs = [16384.0, 8192.0, 4096.0, 2048.0][scale]

        ax /= gs
        ay /= gs
        az /= gs

        # Read back the degrees per second rate and
        # use it to compensate the self.reading to dps
        scale = (self.read(ICM20948_GYRO_CONFIG_1) & 0x06) >> 1

        # scale ranges from section 3.1 of the datasheet
        dps = [131, 65.5, 32.8, 16.4][scale]

        gx /= dps
        gy /= dps
        gz /= dps

        return gx, gy, gz
    
    def read_magnetometer_data_list(self, timeout=1.0):
        self.mag_write(AK09916_CNTL2, 0x01)  # Trigger single measurement
        t_start = time.time()
        while not self.magnetometer_ready():
            if time.time() - t_start > timeout:
                raise RuntimeError("Timeout waiting for Magnetometer Ready")
            time.sleep(0.00001)

        data = self.mag_read_bytes(AK09916_HXL, 6)

        # Read ST2 to confirm self.read finished,
        # needed for continuous modes
        # self.mag_read(AK09916_ST2)

        x, y, z = struct.unpack("<hhh", bytearray(data))
        
        # Scale for magnetic flux density "uT"
        # from section 3.3 of the datasheet
        # This value is constant
        x *= 0.15
        y *= 0.15
        z *= 0.15
        
        magnetometer_status = [x,y,z]

        return magnetometer_status
    
    def read_accelerometer_data_list(self):
        self.bank(0)
        data = self.read_bytes(ICM20948_ACCEL_XOUT_H, 12)

        ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhh", bytearray(data))

        self.bank(2)

        # Read accelerometer full scale range and
        # use it to compensate the self.reading to gs
        scale = (self.read(ICM20948_ACCEL_CONFIG) & 0x06) >> 1

        # scale ranges from section 3.2 of the datasheet
        gs = [16384.0, 8192.0, 4096.0, 2048.0][scale]

        ax /= gs
        ay /= gs
        az /= gs
        
        accelerometer_status = [ax,ay,az]
        
        # Read back the degrees per second rate and
        # use it to compensate the self.reading to dps
        scale = (self.read(ICM20948_GYRO_CONFIG_1) & 0x06) >> 1

        # scale ranges from section 3.1 of the datasheet
        dps = [131, 65.5, 32.8, 16.4][scale]

        gx /= dps
        gy /= dps
        gz /= dps

        return accelerometer_status
    
    def read_gyro_data_list(self):
        self.bank(0)
        data = self.read_bytes(ICM20948_ACCEL_XOUT_H, 12)

        ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhh", bytearray(data))

        self.bank(2)
        
        # Read accelerometer full scale range and
        # use it to compensate the self.reading to gs
        scale = (self.read(ICM20948_ACCEL_CONFIG) & 0x06) >> 1

        # scale ranges from section 3.2 of the datasheet
        gs = [16384.0, 8192.0, 4096.0, 2048.0][scale]

        ax /= gs
        ay /= gs
        az /= gs

        # Read back the degrees per second rate and
        # use it to compensate the self.reading to dps
        scale = (self.read(ICM20948_GYRO_CONFIG_1) & 0x06) >> 1

        # scale ranges from section 3.1 of the datasheet
        dps = [131, 65.5, 32.8, 16.4][scale]

        gx /= dps
        gy /= dps
        gz /= dps
        
        gyro_status = [gx,gy,gz]

        return gyro_status

    def set_accelerometer_sample_rate(self, rate=125):
        """Set the accelerometer sample rate in Hz."""
        self.bank(2)
        # 125Hz - 1.125 kHz / (1 + rate)
        rate = int((1125.0 / rate) - 1)
        # TODO maybe use struct to pack and then write_bytes
        self.write(ICM20948_ACCEL_SMPLRT_DIV_1, (rate >> 8) & 0xff)
        self.write(ICM20948_ACCEL_SMPLRT_DIV_2, rate & 0xff)

    def set_accelerometer_full_scale(self, scale=16):
        """Set the accelerometer fulls cale range to +- the supplied value."""
        self.bank(2)
        value = self.read(ICM20948_ACCEL_CONFIG) & 0b11111001
        value |= {2: 0b00, 4: 0b01, 8: 0b10, 16: 0b11}[scale] << 1
        self.write(ICM20948_ACCEL_CONFIG, value)

    def set_accelerometer_low_pass(self, enabled=True, mode=5):
        """Configure the accelerometer low pass filter."""
        self.bank(2)
        value = self.read(ICM20948_ACCEL_CONFIG) & 0b10001110
        if enabled:
            value |= 0b1
        value |= (mode & 0x07) << 4
        self.write(ICM20948_ACCEL_CONFIG, value)

    def set_gyro_sample_rate(self, rate=125):
        """Set the gyro sample rate in Hz."""
        self.bank(2)
        # 125Hz sample rate - 1.125 kHz / (1 + rate)
        rate = int((1125.0 / rate) - 1)
        self.write(ICM20948_GYRO_SMPLRT_DIV, rate)

    def set_gyro_full_scale(self, scale=250):
        """Set the gyro full scale range to +- supplied value."""
        self.bank(2)
        value = self.read(ICM20948_GYRO_CONFIG_1) & 0b11111001
        value |= {250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11}[scale] << 1
        self.write(ICM20948_GYRO_CONFIG_1, value)

    def set_gyro_low_pass(self, enabled=True, mode=5):
        """Configure the gyro low pass filter."""
        self.bank(2)
        value = self.read(ICM20948_GYRO_CONFIG_1) & 0b10001110
        if enabled:
            value |= 0b1
        value |= (mode & 0x07) << 4
        self.write(ICM20948_GYRO_CONFIG_1, value)

    def read_temperature(self):
        """Property to read the current IMU temperature"""
        # PWR_MGMT_1 defaults to leave temperature enabled
        self.bank(0)
        temp_raw_bytes = self.read_bytes(ICM20948_TEMP_OUT_H, 2)
        temp_raw = struct.unpack('>h', bytearray(temp_raw_bytes))[0]
        temperature_deg_c = ((temp_raw - ICM20948_ROOM_TEMP_OFFSET) / ICM20948_TEMPERATURE_SENSITIVITY) + ICM20948_TEMPERATURE_DEGREES_OFFSET
        return temperature_deg_c

    def __init__(self, i2c_addr=I2C_ADDR, i2c_bus=None):
        self._bank = -1
        self._addr = i2c_addr

        if i2c_bus is None:
            from smbus import SMBus
            self._bus = SMBus(1)
        else:
            self._bus = i2c_bus

        self.bank(0)
        if not self.read(ICM20948_WHO_AM_I) == CHIP_ID:
            raise RuntimeError("Unable to find ICM20948")

        self.write(ICM20948_PWR_MGMT_1, 0x80)
        time.sleep(0.01)
        self.write(ICM20948_PWR_MGMT_1, 0x01)
        self.write(ICM20948_PWR_MGMT_2, 0x00)

        self.bank(2)

        self.set_gyro_sample_rate(100)
        self.set_gyro_low_pass(enabled=True, mode=5)
        self.set_gyro_full_scale(250)

        self.set_accelerometer_sample_rate(125)
        self.set_accelerometer_low_pass(enabled=True, mode=5)
        self.set_accelerometer_full_scale(16)

        self.bank(0)
        self.write(ICM20948_INT_PIN_CFG, 0x30)

        self.bank(3)
        self.write(ICM20948_I2C_MST_CTRL, 0x4D)
        self.write(ICM20948_I2C_MST_DELAY_CTRL, 0x01)

        if not self.mag_read(AK09916_WIA) == AK09916_CHIP_ID:
            raise RuntimeError("Unable to find AK09916")

        # Reset the magnetometer
        self.mag_write(AK09916_CNTL3, 0x01)
        while self.mag_read(AK09916_CNTL3) == 0x01:
            time.sleep(0.0001)

    def magnet_calibrate() :        

        txt_PATH = 'MPUlog.txt'
        #MPUlog.txtの最後の３列がmG（ミリガウス）単位の地磁気データx、y、zとなっている
        data=np.loadtxt(txt_PATH, delimiter=',')

        #地磁気データ抽出
        magx=-data[0]
        magy=data[1]
        magz=-data[2]

        #最小二乗法
        #正規方程式の作成
        #1 x2
        _x4   =sum(magx**4)
        _x2y2 =sum(magx**2*magy**2)
        _x2z2 =sum(magx**2*magz**2)
        _2x3y =sum(2*magx**3*magy)
        _2x2yz=sum(2*magx**2*magy*magz)
        _2x3z =sum(2*magx**3*magz)
        _x3   =sum(magx**3)
        _x2y  =sum(magx**2*magy)
        _x2z  =sum(magx**2*magz)
        #2 y2
        _y4   =sum(magy**4)
        _y2z2 =sum(magy**2*magz**2)
        _2xy3 =sum(2*magx*magy**3)
        _2y3z=sum(2*magy**3*magz)
        _2xy2z=sum(2*magx*magy**2*magz)
        _xy2  =sum(magx*magy**2)
        _y3   =sum(magy**3)
        _y2z  =sum(magy**2*magz)
        #3 z2
        _z4   =sum(magz**4)
        _2xyz2=sum(2*magx*magy*magz**2)
        _2yz3=sum(2*magy*magz**3)
        _2xz3 =sum(2*magx*magz**3)
        _xz2  =sum(magx*magz**2)
        _yz2  =sum(magy*magz**2)
        _z3   =sum(magz**3)
        #4 xy
        _2x2y2=sum(2*magx**2*magy**2)
        _2xy2z=sum(2*magx*magy**2*magz)
        _2x2yz=sum(2*magx**2*magy*magz)
        _x2y  =sum(magx**2*magy)
        _xy2  =sum(magx*magy**2)
        _xyz  =sum(magx*magy*magz)
        #5 yz
        _2y2z2=sum(2*magy**2*magz**2)
        _2xyz2=sum(2*magx*magy*magz**2)
        _xyz  =sum(magx*magy*magz)
        _y2z  =sum(magy**2*magz)
        _yz2  =sum(magy*magz**2)
        #6 xz
        _2x2z2=sum(2*magx**2*magz**2)
        _x2z  =sum(magx**2*magz)
        _xyz  =sum(magx*magy*magz)
        _xz2  =sum(magx*magz**2)
        #7 x
        _x2   =sum(magx**2)
        _xy   =sum(magx*magy)
        _xz   =sum(magx*magz)
        #8 y
        _y2   =sum(magy**2)
        _yz   =sum(magy*magz)
        #9 z
        _z2   =sum(magz**2)
        #b
        _x=sum(magx)
        _y=sum(magy)
        _z=sum(magz)

        #正規行列
        M=np.matrix([[     _x4,    _x2y2,    _x2z2,    _2x3y,   _2x2yz,  _2x3z,  _x3, _x2y, _x2z],
                     [   _x2y2,      _y4,    _y2z2,    _2xy3,    _2y3z, _2xy2z, _xy2,  _y3, _y2z],
                     [   _x2z2,    _y2z2,      _z4,   _2xyz2,    _2yz3,  _2xz3, _xz2, _yz2,  _z3],
                     [ _2x3y/2,  _2xy3/2, _2xyz2/2,   _2x2y2,   _2xy2z, _2x2yz, _x2y, _xy2, _xyz],
                     [_2x2yz/2,  _2y3z/2,  _2yz3/2, _2xy2z/2,   _2y2z2, _2xyz2, _xyz, _y2z, _yz2],
                     [ _2x3z/2, _2xy2z/2,  _2xz3/2, _2x2yz/2, _2xyz2/2, _2x2z2, _x2z, _xyz, _xz2],
                     [     _x3,     _xy2,     _xz2,     _x2y,     _xyz,   _x2z,  _x2,  _xy,  _xz],
                     [    _x2y,      _y3,     _yz2,     _xy2,     _y2z,   _xyz,  _xy,  _y2,  _yz],
                     [    _x2z,     _y2z,      _z3,     _xyz,     _yz2,   _xz2,  _xz,  _yz,  _z2]
                    ])

        b=np.matrix([_x2, _y2, _z2, _xy, _yz, _xz, _x, _y, _z]).T

        #楕円体パラメータの算出
        x=np.linalg.inv(M)*(-b)

        #print('楕円体パラメータ')
        #print(x)

        a11=x[0,0]
        a22=x[1,0]
        a33=x[2,0]
        a12=x[3,0]
        a23=x[4,0]
        a13=x[5,0]
        b1 =x[6,0]
        b2 =x[7,0]
        b3 =x[8,0]

        #楕円体2次項パラメータ行列
        A=np.matrix([[a11,a12,a13],[a12,a22,a23],[a13,a23,a33]])

        #2次項パラメータ行列の固有値と固有ベクトルを求める
        lbda,v=np.linalg.eig(A)

        #print('固有値・固有ベクトル')
        #print(lbda)
        #print(v)

        #回転・並行移動
        #固有値と基底ベクトル（固有ベクトル）の並べ替え
        vt=v.T
        vx=vt[1].T
        vy=vt[2].T
        vz=vt[0].T
        lbdx=lbda[1]
        lbdy=lbda[2]
        lbdz=lbda[0]

        #1次項のパラメータベクトル
        B=np.matrix([b1,b2,b3])

        #対角化行列(回転行列)
        P=np.hstack([vx,vy,vz])
        #print('回転行列')
        #print(P.T)

        #楕円体の中心座標算出
        x0=(-B*vx/2/lbdx)[0,0]
        y0=(-B*vy/2/lbdy)[0,0]
        z0=(-B*vz/2/lbdz)[0,0]

        print('中心座標')
        print(x0,y0,z0)
        out = [x0,y0,z0]

        #球に変換
        W=(B*vx)[0,0]**2/4/lbdx + (B*vy)[0,0]**2/4/lbdy + (B*vz)[0,0]**2/4/lbdz -1 
        print('W')
        print(W)

        sx=np.sqrt(lbdx/W)
        sy=np.sqrt(lbdy/W)
        sz=np.sqrt(lbdz/W)
        smax=max(sx,sy,sz)
        sx=sx/smax
        sy=sy/smax
        sz=sz/smax

        #print('拡大係数')
        #print(sx, sy, sz)


        #移動計算のためデータ整形
        mag2=np.vstack([magx,magy,magz])

        #回転
        mag2=P.T*mag2

        #並行移動
        magx2=np.array(mag2)[0]-x0
        magy2=np.array(mag2)[1]-y0
        magz2=np.array(mag2)[2]-z0

        #拡大縮小
        magx2=sx*magx2
        magy2=sy*magy2
        magz2=sz*magz2
    
        txt_PATH = 'magnet_calibrate.txt'
        f = open(txt_PATH, 'w', encoding='UTF-8')
        out= str(out)
        f.write(out[1:len(out)-1:1])
        f.close()
    
        return 0

    #機体の方角を取得(MPU)
    def get_myangle() :
        #!データが真円になるように補正する必要あり
        Px = float(magnet_calibrate_list[0])
        Py = float(magnet_calibrate_list[1])
        #Px =-60                            
        #Py =250
        magnetlist = imu.read_magnetometer_data_list()
        #magnetlist = [100,0]             #ダミーデータ
        magnet_x  = float(magnetlist[0])
        magnet_y  = float(magnetlist[1])

        x = magnet_x - Px
        y = magnet_y - Py

        myangle = (0 - math.degrees(math.atan2(y,x))) % 360

        return myangle

if __name__ == "__main__":
    imu = ICM20948()

    while True:
        x, y, z = imu.read_magnetometer_data()
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()

        print("""
Accel: {:05.2f} {:05.2f} {:05.2f}
Gyro:  {:05.2f} {:05.2f} {:05.2f}
Mag:   {:05.2f} {:05.2f} {:05.2f}""".format(
            ax, ay, az, gx, gy, gz, x, y, z
        ))
        
        print("x, y, z,ax, ay, az, gx, gy, gz")
        print(imu.read_accelerometer_data())
        print(imu.read_gyro_data())
        
        f = open('MPUlog.txt', 'a', encoding='UTF-8')
        f.writelines(str(imu.read_accelerometer_data()))
        f.writelines("\n")
        f.writelines(str(imu.read_gyro_data()))
        f.writelines("\n")
        f.writelines(str(imu.read_magnetometer_data()))
        f.writelines("\n")
        f.writelines(str(imu.read_accelerometer_data_list()))
        f.writelines("\n")
        f.writelines(str(imu.read_gyro_data_list()))
        f.writelines("\n")
        f.writelines(str(imu.read_magnetometer_data_list()))
        f.writelines("\n")
        f.writelines("\n")
        f.close()
        
        a = open('accelerolog.txt', 'a', encoding='UTF-8')
        a.writelines(str(imu.read_accelerometer_data()))
        a.writelines("\n")
        a.writelines(str(imu.read_accelerometer_data_list()))
        a.writelines("\n")
        a.writelines("\n")
        a.close()
        
        
        g = open('gyrolog.txt', 'a', encoding='UTF-8')
        g.writelines(str(imu.read_gyro_data()))
        g.writelines("\n")
        g.writelines(str(imu.read_gyro_data_list()))
        g.writelines("\n")
        g.writelines("\n")
        g.close()
        
        m = open('magnetolog.txt', 'a', encoding='UTF-8')
        m.writelines(str(imu.read_magnetometer_data()))
        m.writelines("\n")
        m.writelines(str(imu.read_magnetometer_data_list()))
        m.writelines("\n")
        m.writelines("\n")
        m.close()

        time.sleep(10)
        
        