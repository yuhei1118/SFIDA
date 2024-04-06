# -*- coding: utf-8 -*-
"""
Created on Wed May 24 01:36:00 2022

@author: 佐藤　裕平
"""

#モジュールインポート
from __future__ import print_function
import time
import datetime
import math
import numpy as np
import pigpio
import cv2
import smbus2 as smbus
import serial
import spidev
import picamera
import sys
import qwiic_titan_gps
import csv
from gpiozero import PWMLED
import binascii
import logging
import struct
import Adafruit_GPIO as I2C
#設定値決定

cycle_time = 1.1                  #機体が1回転する時間（機体，モーター，バッテリー等によって機体が一回転する時間が違うので10回程度サンプルを取って平均を割り出す）
preset_altiude = 10                #着地高度設定(気圧の変化が激しい場合は余裕をもって高めに設定する)
preset_altiude2 = 5               #上昇高度設定
preset_goal_distanc = 10          #誘導方法切り替え距離(天気や画像認識精度によって変更する)
go_time = 10                      #GPS誘導時直進時間(予想される落下分散から機体の角度修正をする間隔を決める能代だと5秒くらいに設定していた)
preset_pres = 882.61              #ゴール地点気圧(setup.py製作以降設定しなくても良くなった) 
career_time = 4                  #!電熱線燃焼時間
magnet_calibrate_list = [-70,220] #!地磁気補正データ

LEFT_TOP_M1 = 4 # lt      #モーター・電熱線・通信電源ピンを割り当て(GPIOピンの番号を設定する常時highのピンではなくlowになってるピンを使うように設計しよう)
LEFT_TOP_M2 = 17
RIGHT_TOP_M1 = 27 # rt
RIGHT_TOP_M2 = 22
CAREER_CUT = 25
IM_PIN = 24

V_REF = 3.29476           # 光センサinput Voltage
CHN = 0                   # 光センサ接続チャンネル

#BME280
bus_number  = 1
start = time.time()
bus = smbus.SMBus(bus_number)
i2c_address = 0x76

digT = []
digP = []
digH = []

t_fine = 0.0

def writeReg(reg_address, data):
    bus.write_byte_data(i2c_address,reg_address,data)

def get_calib_param():
    calib = []
    for i in range (0x88,0x88+24):
        calib.append(bus.read_byte_data(i2c_address,i))
    calib.append(bus.read_byte_data(i2c_address,0xA1))
    for i in range (0xE1,0xE1+7):
        calib.append(bus.read_byte_data(i2c_address,i))

    digT.append((calib[1] << 8) | calib[0])
    digT.append((calib[3] << 8) | calib[2])
    digT.append((calib[5] << 8) | calib[4])
    digP.append((calib[7] << 8) | calib[6])
    digP.append((calib[9] << 8) | calib[8])
    digP.append((calib[11]<< 8) | calib[10])
    digP.append((calib[13]<< 8) | calib[12])
    digP.append((calib[15]<< 8) | calib[14])
    digP.append((calib[17]<< 8) | calib[16])
    digP.append((calib[19]<< 8) | calib[18])
    digP.append((calib[21]<< 8) | calib[20])
    digP.append((calib[23]<< 8) | calib[22])
    digH.append( calib[24] )
    digH.append((calib[26]<< 8) | calib[25])
    digH.append( calib[27] )
    digH.append((calib[28]<< 4) | (0x0F & calib[29]))
    digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
    digH.append( calib[31] )

    for i in range(1,2):
        if digT[i] & 0x8000:
            digT[i] = (-digT[i] ^ 0xFFFF) + 1

    for i in range(1,8):
        if digP[i] & 0x8000:
            digP[i] = (-digP[i] ^ 0xFFFF) + 1

    for i in range(0,6):
        if digH[i] & 0x8000:
            digH[i] = (-digH[i] ^ 0xFFFF) + 1

def read_Data():
    data = []
    for i in range (0xF7, 0xF7+8):
        data.append(bus.read_byte_data(i2c_address,i))
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw  = (data[6] << 8)  |  data[7]

        #compensate_T(temp_raw)
        #compensate_P(pres_raw)
        #compensate_H(hum_raw)
    t = compensate_T(temp_raw)
    p = compensate_P(pres_raw)
    h = compensate_H(hum_raw)
    
    return   p + "," + t +"," + h
#気圧読み出し
def read_pres() :
    data = []
    for i in range (0xF7, 0xF7+8):
        data.append(bus.read_byte_data(i2c_address,i))
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    p = compensate_P(pres_raw)
    #p = 1013.25 #ダミーデータ
    #戻り値は取得した気圧(hp)
    return p
#気温読み出し
def read_temp() :
    data = []
    for i in range (0xF7, 0xF7+8):
        data.append(bus.read_byte_data(i2c_address,i))
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    t = compensate_T(temp_raw)
    #t = 25 #ダミーデータ
    #戻り値は取得した気温(℃)
    return t
#高度計算 引数は基準気圧(ゴールの気圧)
def get_altitude(pf) :
    p0    = 1013.25
    p0fix = pf
    p1    = float(read_pres())
    t1    = float(read_temp())
    T     = t1 + 273.15
    P     = p0 / p1
    Pfix  = p0 / p0fix
    v1    = 1 / 5.257
    v2    = pow(P,v1)
    v3    = pow(Pfix,v1)
    v4    = v2 - v3
    v5    = v4 * T
    h     = v5 / 0.0065
    #h = 0
    #戻り値は取得した高度(m)
    return h

def compensate_P(adc_P):
    global  t_fine
    pressure = 0.0

    v1 = (t_fine / 2.0) - 64000.0
    v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
    v2 = v2 + ((v1 * digP[4]) * 2.0)
    v2 = (v2 / 4.0) + (digP[3] * 65536.0)
    v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144
    v1 = ((32768 + v1) * digP[0]) / 32768

    if v1 == 0:
        return 0
    pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
    if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
    else:
        pressure = (pressure / v1) * 2
    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0
    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)

    #print "pressure : %7.2f hPa" % (pressure/100)
    return "%7.2f" % (pressure/100)

def compensate_T(adc_T):
    global t_fine
    v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
    t_fine = v1 + v2
    temperature = t_fine / 5120.0
    #print "temp : %-6.2f ℃" % (temperature)
    return "%.2f" % (temperature)

def compensate_H(adc_H):
    global t_fine
    var_h = t_fine - 76800.0
    if var_h != 0:
        var_h = (adc_H - (digH[3] * 64.0 + digH[4]/16384.0 * var_h)) * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)))
    else:
        return 0
    var_h = var_h * (1.0 - digH[0] * var_h / 524288.0)
    if var_h > 100.0:
        var_h = 100.0
    elif var_h < 0.0:
        var_h = 0.0
    #print "hum : %6.2f ％" % (var_h)
    return "%.2f" % (var_h)
#BME280の基本設定
def BME_setup():
#        open('/home/pi/row.csv', 'w')
    osrs_t = 1            #Temperature oversampling x 1
    osrs_p = 1            #Pressure oversampling x 1
    osrs_h = 1            #Humidity oversampling x 1
    mode   = 3            #Normal mode
    t_sb   = 5            #Tstandby 1000ms
    filter = 0            #Filter off
    spi3w_en = 0            #3-wire SPI Disable

    ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
    config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
    ctrl_hum_reg  = osrs_h

    writeReg(0xF2,ctrl_hum_reg)
    writeReg(0xF4,ctrl_meas_reg)
    writeReg(0xF5,config_reg)

#緑GPS

#データを取得するファイル名
UART_PATH = '/dev/serial0'
#取得したデータを書き込む場所
GPS_Realtime_txt_PATH = "gps_realtime.txt"

#GPSデータを取得する関数
def read_gps():
    global pro_count
    global GPS_Realtime_txt_PATH
    Main_State = 0
    GPS_raw_Array = i_GPS_UART_getval()
#    print(GPS_raw_Array)
    GPS_Array = r_NMEA_Decorder(GPS_raw_Array)
    if GPS_Array != 0:
        Main_State = t_TXT_Write_module(GPS_Array,GPS_Realtime_txt_PATH)
    #戻り値はリスト形式で[longitude,latitude](DMM系で西経南緯はマイナスで表示される)
    return Main_State
#UARTパスからデータ読み出し
def i_GPS_UART_getval():
    global pro_count
    global UART_PATH
    pro_count = "i"
    ser = serial.Serial(UART_PATH,9600,timeout = 0.5)
    Data = ser.readline()
    ser.close
    #Data = 	["'$GPGGA", '204628.000', '4222.8112', 'N', '14102.0097', 'E', '2', '11', '0.94', '61.5', 'M', '34.5', 'M', '', "*5C\\r\\n'"]
    Data = str(Data)
    Data = Data[1:len(Data)]
    time.sleep(0.1)
    GPS_Array = Data.split(',')
    return GPS_Array
#生データを人間が使える形にデコード
def r_NMEA_Decorder(GPS_Array):
    global pro_count
    pro_count = "r"
    DataType = 0
    GGA_DMM_latitude = 2
    GGA_DMM_longitude = 4
    lalo = [0,0]
    if GPS_Array[DataType] == "'$GPGGA":
        lalo[0] = sub_s_DD_DMM_converter_la(GPS_Array[GGA_DMM_latitude])
        lalo[1] = sub_s_DD_DMM_converter_lo(GPS_Array[GGA_DMM_longitude])

#        GPS_Array_a = list(GPS_Array[GGA_DMM_latitude])

#        GPS_Array_a[0] = " "
#        GPS_Array_a[1] = " "
#        GPS_Array_a[len(GPS_Array_a)-1] = " "
#        GPS_Array_a[len(GPS_Array_a)-2] = " "
#        GPS_Array_b = list(GPS_Array[GGA_DMM_longitude])
#        print(GPS_Array_a)
#        GPS_Array_b[0] = " "
#        GPS_Array_b[1] = " "
#        GPS_Array_b[len(GPS_Array_b)-1] = " "
#        GPS_Array_b[len(GPS_Array_b)-2] = " "

#        print(GPS_Array_b)
#        lalo[0] = sub_s_DD_DMM_converter(''.join([str(i) for i in GPS_Array_a]))
#        lalo[1] = sub_s_DD_DMM_converter(''.join([str(i) for i in GPS_Array_b]))

    if lalo[0] != 0 and lalo[1] != 0:
        return lalo
    return 0
#la座標形式変換 DD→DMM
def sub_s_DD_DMM_converter_la(DMM_val):
    global pro_count
    pro_count = "s"

    y_DD = 0
    if DMM_val != "":
        DMM_val = float(DMM_val)/100
        DMM_val = round(DMM_val , 6)
        y_str = str(DMM_val)
#        print(y_str,"a")
        #y_float = float(DMM_val/100)
        y_min = float(y_str[3:len(y_str):1])/60*100 / 1e6
#        print(len(y_str))
        if len(y_str) != 9:
            y_min *= 10
#        print(y_min , "n")
        y_DD  = float(y_str[0:3:1]) + y_min
        y_DD = round(y_DD , 5)
    return y_DD
#lo座標形式変換 DD→DMM
def sub_s_DD_DMM_converter_lo(DMM_val):
    global pro_count
    pro_count = "s"
    y_DD = 0
    if DMM_val != "":
        DMM_val = float(DMM_val)/100
        DMM_val = round(DMM_val , 7)
        y_str = str(DMM_val)
#        print(y_str,"a")
        #y_float = float(DMM_val/100)
        y_min = float(y_str[4:len(y_str):1])/60*100 / 1e6
#        print(len(y_str))
        if len(y_str) != 10:
            y_min *= 10
#        print(y_min , "n")
        y_DD  = float(y_str[0:3:1]) + y_min
        y_DD = round(y_DD , 5)
    return y_DD
#テキスト書き込み用関数
def t_TXT_Write_module(GPS_Array,GPS_txt_PATH):

    if GPS_Array[0] != 0 and GPS_Array[1] != 0:

        GPS_Array = str(GPS_Array)
        with open(GPS_txt_PATH, "w") as lf :
            gps_write =lf.writelines(GPS_Array[1:len(GPS_Array)-1:1])
            lf.close
    return GPS_Array

#赤GPS
#用途は緑GPSと同様
def read_rgps() :
    global GPS_Realtime_txt_PATH
    qwiicGPS = qwiic_titan_gps.QwiicTitanGps()
    
    if qwiicGPS.connected is False:
        print("Could not connect to to the SparkFun GPS Unit. Double check that\
              it's wired correctly.", file=sys.stderr)
        return

    qwiicGPS.begin()
    
    if qwiicGPS.get_nmea_data() is True:
        GPS_Array = [qwiicGPS.gnss_messages['Latitude'], qwiicGPS.gnss_messages['Longitude']]
        if GPS_Array != 0:
            Main_State = t_TXT_Write_module(GPS_Array,GPS_Realtime_txt_PATH)
            return Main_State

#AE-BNO055-BO
# Adafruit BNO055 Absolute Orientation Sensor Library
# Copyright (c) 2015 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

logger = logging.getLogger(__name__)

class BNO055:
    
    # I2C addresses
    BNO055_ADDRESS_A                     = 0x28
    BNO055_ADDRESS_B                     = 0x29
    BNO055_ID                            = 0xA0
    
    # Page id register definition
    BNO055_PAGE_ID_ADDR                  = 0X07
    
    # PAGE0 REGISTER DEFINITION START
    BNO055_CHIP_ID_ADDR                  = 0x00
    BNO055_ACCEL_REV_ID_ADDR             = 0x01
    BNO055_MAG_REV_ID_ADDR               = 0x02
    BNO055_GYRO_REV_ID_ADDR              = 0x03
    BNO055_SW_REV_ID_LSB_ADDR            = 0x04
    BNO055_SW_REV_ID_MSB_ADDR            = 0x05
    BNO055_BL_REV_ID_ADDR                = 0X06
    
    # Accel data register
    BNO055_ACCEL_DATA_X_LSB_ADDR         = 0X08
    BNO055_ACCEL_DATA_X_MSB_ADDR         = 0X09
    BNO055_ACCEL_DATA_Y_LSB_ADDR         = 0X0A
    BNO055_ACCEL_DATA_Y_MSB_ADDR         = 0X0B
    BNO055_ACCEL_DATA_Z_LSB_ADDR         = 0X0C
    BNO055_ACCEL_DATA_Z_MSB_ADDR         = 0X0D
    
    # Mag data register
    BNO055_MAG_DATA_X_LSB_ADDR           = 0X0E
    BNO055_MAG_DATA_X_MSB_ADDR           = 0X0F
    BNO055_MAG_DATA_Y_LSB_ADDR           = 0X10
    BNO055_MAG_DATA_Y_MSB_ADDR           = 0X11
    BNO055_MAG_DATA_Z_LSB_ADDR           = 0X12
    BNO055_MAG_DATA_Z_MSB_ADDR           = 0X13
    
    # Gyro data registers
    BNO055_GYRO_DATA_X_LSB_ADDR          = 0X14
    BNO055_GYRO_DATA_X_MSB_ADDR          = 0X15
    BNO055_GYRO_DATA_Y_LSB_ADDR          = 0X16
    BNO055_GYRO_DATA_Y_MSB_ADDR          = 0X17
    BNO055_GYRO_DATA_Z_LSB_ADDR          = 0X18
    BNO055_GYRO_DATA_Z_MSB_ADDR          = 0X19
    
    # Euler data registers
    BNO055_EULER_H_LSB_ADDR              = 0X1A
    BNO055_EULER_H_MSB_ADDR              = 0X1B
    BNO055_EULER_R_LSB_ADDR              = 0X1C
    BNO055_EULER_R_MSB_ADDR              = 0X1D
    BNO055_EULER_P_LSB_ADDR              = 0X1E
    BNO055_EULER_P_MSB_ADDR              = 0X1F
    
    # Quaternion data registers
    BNO055_QUATERNION_DATA_W_LSB_ADDR    = 0X20
    BNO055_QUATERNION_DATA_W_MSB_ADDR    = 0X21
    BNO055_QUATERNION_DATA_X_LSB_ADDR    = 0X22
    BNO055_QUATERNION_DATA_X_MSB_ADDR    = 0X23
    BNO055_QUATERNION_DATA_Y_LSB_ADDR    = 0X24
    BNO055_QUATERNION_DATA_Y_MSB_ADDR    = 0X25
    BNO055_QUATERNION_DATA_Z_LSB_ADDR    = 0X26
    BNO055_QUATERNION_DATA_Z_MSB_ADDR    = 0X27
    
    # Linear acceleration data registers
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  = 0X28
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR  = 0X29
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR  = 0X2A
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR  = 0X2B
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR  = 0X2C
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR  = 0X2D
    
    # Gravity data registers
    BNO055_GRAVITY_DATA_X_LSB_ADDR       = 0X2E
    BNO055_GRAVITY_DATA_X_MSB_ADDR       = 0X2F
    BNO055_GRAVITY_DATA_Y_LSB_ADDR       = 0X30
    BNO055_GRAVITY_DATA_Y_MSB_ADDR       = 0X31
    BNO055_GRAVITY_DATA_Z_LSB_ADDR       = 0X32
    BNO055_GRAVITY_DATA_Z_MSB_ADDR       = 0X33
    
    # Temperature data register
    BNO055_TEMP_ADDR                     = 0X34
    
    # Status registers
    BNO055_CALIB_STAT_ADDR               = 0X35
    BNO055_SELFTEST_RESULT_ADDR          = 0X36
    BNO055_INTR_STAT_ADDR                = 0X37
    
    BNO055_SYS_CLK_STAT_ADDR             = 0X38
    BNO055_SYS_STAT_ADDR                 = 0X39
    BNO055_SYS_ERR_ADDR                  = 0X3A
    
    # Unit selection register
    BNO055_UNIT_SEL_ADDR                 = 0X3B
    BNO055_DATA_SELECT_ADDR              = 0X3C
    
    # Mode registers
    BNO055_OPR_MODE_ADDR                 = 0X3D
    BNO055_PWR_MODE_ADDR                 = 0X3E
    
    BNO055_SYS_TRIGGER_ADDR              = 0X3F
    BNO055_TEMP_SOURCE_ADDR              = 0X40
    
    # Axis remap registers
    BNO055_AXIS_MAP_CONFIG_ADDR          = 0X41
    BNO055_AXIS_MAP_SIGN_ADDR            = 0X42
    
    # Axis remap values
    AXIS_REMAP_X                         = 0x00
    AXIS_REMAP_Y                         = 0x01
    AXIS_REMAP_Z                         = 0x02
    AXIS_REMAP_POSITIVE                  = 0x00
    AXIS_REMAP_NEGATIVE                  = 0x01
    
    # SIC registers
    BNO055_SIC_MATRIX_0_LSB_ADDR         = 0X43
    BNO055_SIC_MATRIX_0_MSB_ADDR         = 0X44
    BNO055_SIC_MATRIX_1_LSB_ADDR         = 0X45
    BNO055_SIC_MATRIX_1_MSB_ADDR         = 0X46
    BNO055_SIC_MATRIX_2_LSB_ADDR         = 0X47
    BNO055_SIC_MATRIX_2_MSB_ADDR         = 0X48
    BNO055_SIC_MATRIX_3_LSB_ADDR         = 0X49
    BNO055_SIC_MATRIX_3_MSB_ADDR         = 0X4A
    BNO055_SIC_MATRIX_4_LSB_ADDR         = 0X4B
    BNO055_SIC_MATRIX_4_MSB_ADDR         = 0X4C
    BNO055_SIC_MATRIX_5_LSB_ADDR         = 0X4D
    BNO055_SIC_MATRIX_5_MSB_ADDR         = 0X4E
    BNO055_SIC_MATRIX_6_LSB_ADDR         = 0X4F
    BNO055_SIC_MATRIX_6_MSB_ADDR         = 0X50
    BNO055_SIC_MATRIX_7_LSB_ADDR         = 0X51
    BNO055_SIC_MATRIX_7_MSB_ADDR         = 0X52
    BNO055_SIC_MATRIX_8_LSB_ADDR         = 0X53
    BNO055_SIC_MATRIX_8_MSB_ADDR         = 0X54
    
    # Accelerometer Offset registers
    ACCEL_OFFSET_X_LSB_ADDR              = 0X55
    ACCEL_OFFSET_X_MSB_ADDR              = 0X56
    ACCEL_OFFSET_Y_LSB_ADDR              = 0X57
    ACCEL_OFFSET_Y_MSB_ADDR              = 0X58
    ACCEL_OFFSET_Z_LSB_ADDR              = 0X59
    ACCEL_OFFSET_Z_MSB_ADDR              = 0X5A
    
    # Magnetometer Offset registers
    MAG_OFFSET_X_LSB_ADDR                = 0X5B
    MAG_OFFSET_X_MSB_ADDR                = 0X5C
    MAG_OFFSET_Y_LSB_ADDR                = 0X5D
    MAG_OFFSET_Y_MSB_ADDR                = 0X5E
    MAG_OFFSET_Z_LSB_ADDR                = 0X5F
    MAG_OFFSET_Z_MSB_ADDR                = 0X60
    
    # Gyroscope Offset register s
    GYRO_OFFSET_X_LSB_ADDR               = 0X61
    GYRO_OFFSET_X_MSB_ADDR               = 0X62
    GYRO_OFFSET_Y_LSB_ADDR               = 0X63
    GYRO_OFFSET_Y_MSB_ADDR               = 0X64
    GYRO_OFFSET_Z_LSB_ADDR               = 0X65
    GYRO_OFFSET_Z_MSB_ADDR               = 0X66
    
    # Radius registers
    ACCEL_RADIUS_LSB_ADDR                = 0X67
    ACCEL_RADIUS_MSB_ADDR                = 0X68
    MAG_RADIUS_LSB_ADDR                  = 0X69
    MAG_RADIUS_MSB_ADDR                  = 0X6A
    
    # Power modes
    POWER_MODE_NORMAL                    = 0X00
    POWER_MODE_LOWPOWER                  = 0X01
    POWER_MODE_SUSPEND                   = 0X02
    
    # Operation mode settings
    OPERATION_MODE_CONFIG                = 0X00
    OPERATION_MODE_ACCONLY               = 0X01
    OPERATION_MODE_MAGONLY               = 0X02
    OPERATION_MODE_GYRONLY               = 0X03
    OPERATION_MODE_ACCMAG                = 0X04
    OPERATION_MODE_ACCGYRO               = 0X05
    OPERATION_MODE_MAGGYRO               = 0X06
    OPERATION_MODE_AMG                   = 0X07
    OPERATION_MODE_IMUPLUS               = 0X08
    OPERATION_MODE_COMPASS               = 0X09
    OPERATION_MODE_M4G                   = 0X0A
    OPERATION_MODE_NDOF_FMC_OFF          = 0X0B
    OPERATION_MODE_NDOF                  = 0X0C

    def __init__(self, rst=None, address=BNO055_ADDRESS_A, i2c=None, gpio=None,
                 serial_port=None, serial_timeout_sec=5, **kwargs):
        # If reset pin is provided save it and a reference to provided GPIO
        # bus (or the default system GPIO bus if none is provided).
        self._rst = rst
        if self._rst is not None:
            if gpio is None:
                import Adafruit_GPIO as GPIO
                gpio = GPIO.get_platform_gpio()
            self._gpio = gpio
            # Setup the reset pin as an output at a high level.
            self._gpio.setup(self._rst, GPIO.OUT)
            self._gpio.set_high(self._rst)
            # Wait a 650 milliseconds in case setting the reset high reset the chip.
            time.sleep(0.65)
        self._serial = None
        self._i2c_device = None
        if serial_port is not None:
            # Use serial communication if serial_port name is provided.
            # Open the serial port at 115200 baud, 8N1.  Add a 5 second timeout
            # to prevent hanging if device is disconnected.
            self._serial = serial.Serial(serial_port, 115200, timeout=serial_timeout_sec,
                                         writeTimeout=serial_timeout_sec)
        else:
            # Use I2C if no serial port is provided.
            # Assume we're using platform's default I2C bus if none is specified.
            if i2c is None:
                import Adafruit_GPIO.I2C as I2C
                i2c = I2C
            # Save a reference to the I2C device instance for later communication.
            self._i2c_device = i2c.get_i2c_device(address, **kwargs)

    def _serial_send(self, command, ack=True, max_attempts=5):
        # Send a serial command and automatically handle if it needs to be resent
        # because of a bus error.  If ack is True then an ackowledgement is
        # expected and only up to the maximum specified attempts will be made
        # to get a good acknowledgement (default is 5).  If ack is False then
        # no acknowledgement is expected (like when resetting the device).
        attempts = 0
        while True:
            # Flush any pending received data to get into a clean state.
            self._serial.flushInput()
            # Send the data.
            self._serial.write(command)
            logger.debug('Serial send: 0x{0}'.format(binascii.hexlify(command)))
            # Stop if no acknowledgment is expected.
            if not ack:
                return
            # Read acknowledgement response (2 bytes).
            resp = bytearray(self._serial.read(2))
            logger.debug('Serial receive: 0x{0}'.format(binascii.hexlify(resp)))
            if resp is None or len(resp) != 2:
                raise RuntimeError('Timeout waiting for serial acknowledge, is the BNO055 connected?')
            # Stop if there's no bus error (0xEE07 response) and return response bytes.
            if not (resp[0] == 0xEE and resp[1] == 0x07):
                return resp
            # Else there was a bus error so resend, as recommended in UART app
            # note at:
            #   http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST-BNO055-AN012-00.pdf
            attempts += 1
            if attempts >=  max_attempts:
                raise RuntimeError('Exceeded maximum attempts to acknowledge serial command without bus error!')

    
    def _write_bytes(self, address, data, ack=True):
        # Write a list of 8-bit values starting at the provided register address.
        if self._i2c_device is not None:
            # I2C write.
            self._i2c_device.writeList(address, data)
        else:
            # Build and send serial register write command.
            command = bytearray(4+len(data))
            command[0] = 0xAA  # Start byte
            command[1] = 0x00  # Write
            command[2] = address & 0xFF
            command[3] = len(data) & 0xFF
            command[4:] = map(lambda x: x & 0xFF, data)
            resp = self._serial_send(command, ack=ack)
            # Verify register write succeeded if there was an acknowledgement.
            if resp[0] != 0xEE and resp[1] != 0x01:
                raise RuntimeError('Register write error: 0x{0}'.format(binascii.hexlify(resp)))
    
    def setExternalCrystalUse(self, useExternalCrystal = True):
        prevMode = self._mode
        self.setMode(BNO055.OPERATION_MODE_CONFIG)
        time.sleep(0.025)
        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x80] if useExternalCrystal else [0])
        time.sleep(0.01)
        self.setMode(prevMode)
        time.sleep(0.02)

    def _write_byte(self, address, value, ack=True):
        # Write an 8-bit value to the provided register address.  If ack is True
        # then expect an acknowledgement in serial mode, otherwise ignore any
        # acknowledgement (necessary when resetting the device).
        if self._i2c_device is not None:
            # I2C write.
            self._i2c_device.write8(address, value)
        else:
            # Build and send serial register write command.
            command = bytearray(5)
            command[0] = 0xAA  # Start byte
            command[1] = 0x00  # Write
            command[2] = address & 0xFF
            command[3] = 1     # Length (1 byte)
            command[4] = value & 0xFF
            resp = self._serial_send(command, ack=ack)
            # Verify register write succeeded if there was an acknowledgement.
            if ack and resp[0] != 0xEE and resp[1] != 0x01:
                raise RuntimeError('Register write error: 0x{0}'.format(binascii.hexlify(resp)))

    def _read_bytes(self, address, length):
        # Read a number of unsigned byte values starting from the provided address.
        if self._i2c_device is not None:
            # I2C read.
            return bytearray(self._i2c_device.readList(address, length))
        else:
            # Build and send serial register read command.
            command = bytearray(4)
            command[0] = 0xAA  # Start byte
            command[1] = 0x01  # Read
            command[2] = address & 0xFF
            command[3] = length & 0xFF
            resp = self._serial_send(command)
            # Verify register read succeeded.
            if resp[0] != 0xBB:
                 raise RuntimeError('Register read error: 0x{0}'.format(binascii.hexlify(resp)))
            # Read the returned bytes.
            length = resp[1]
            resp = bytearray(self._serial.read(length))
            logger.debug('Received: 0x{0}'.format(binascii.hexlify(resp)))
            if resp is None or len(resp) != length:
                raise RuntimeError('Timeout waiting to read data, is the BNO055 connected?')
            return resp

    def _read_byte(self, address):
        # Read an 8-bit unsigned value from the provided register address.
        if self._i2c_device is not None:
            # I2C read.
            return self._i2c_device.readU8(address)
        else:
            return self._read_bytes(address, 1)[0]

    def _read_signed_byte(self, address):
        # Read an 8-bit signed value from the provided register address.
        data = self._read_byte(address)
        if data > 127:
            return data - 256
        else:
            return data

    def _config_mode(self):
        # Enter configuration mode.
        self.set_mode(BNO055.OPERATION_MODE_CONFIG)

    def _operation_mode(self):
        # Enter operation mode to read sensor data.
        self.set_mode(self._mode)

    def begin(self, mode=OPERATION_MODE_NDOF):
        """Initialize the BNO055 sensor.  Must be called once before any other
        BNO055 library functions.  Will return True if the BNO055 was
        successfully initialized, and False otherwise.
        """
        # Save the desired normal operation mode.
        self._mode = mode
        # First send a thow-away command and ignore any response or I2C errors
        # just to make sure the BNO is in a good state and ready to accept
        # commands (this seems to be necessary after a hard power down).
        try:
            self._write_byte(BNO055.BNO055_PAGE_ID_ADDR, 0, ack=False)
        except IOError:
            # Swallow an IOError that might be raised by an I2C issue.  Only do
            # this for this very first command to help get the BNO and board's
            # I2C into a clear state ready to accept the next commands.
            pass
        # Make sure we're in config mode and on page 0.
        self._config_mode()
        self._write_byte(BNO055.BNO055_PAGE_ID_ADDR, 0)
        # Check the chip ID
        bno_id = self._read_byte(BNO055.BNO055_CHIP_ID_ADDR)
        logger.debug('Read chip ID: 0x{0:02X}'.format(bno_id))
        if bno_id != BNO055.BNO055_ID:
            return False
        # Reset the device.
        if self._rst is not None:
            # Use the hardware reset pin if provided.
            # Go low for a short period, then high to signal a reset.
            self._gpio.set_low(self._rst)
            time.sleep(0.01)  # 10ms
            self._gpio.set_high(self._rst)
        else:
            # Else use the reset command.  Note that ack=False is sent because
            # the chip doesn't seem to ack a reset in serial mode (by design?).
            self._write_byte(BNO055.BNO055_SYS_TRIGGER_ADDR, 0x20, ack=False)
        # Wait 650ms after reset for chip to be ready (as suggested
        # in datasheet).
        time.sleep(0.65)
        # Set to normal power mode.
        self._write_byte(BNO055.BNO055_PWR_MODE_ADDR, BNO055.POWER_MODE_NORMAL)
        # Default to internal oscillator.
        self._write_byte(BNO055.BNO055_SYS_TRIGGER_ADDR, 0x0)
        # Enter normal operation mode.
        self._operation_mode()
        return True

    def set_mode(self, mode):
        """Set operation mode for BNO055 sensor.  Mode should be a value from
        table 3-3 and 3-5 of the datasheet:
          http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
        """
        self._write_byte(BNO055.BNO055_OPR_MODE_ADDR, mode & 0xFF)
        # Delay for 30 milliseconds (datsheet recommends 19ms, but a little more
        # can't hurt and the kernel is going to spend some unknown amount of time
        # too).
        time.sleep(0.03)

    def get_revision(self):
        """Return a tuple with revision information about the BNO055 chip.  Will
        return 5 values:
          - Software revision
          - Bootloader version
          - Accelerometer ID
          - Magnetometer ID
          - Gyro ID
        """
        # Read revision values.
        accel = self._read_byte(BNO055.BNO055_ACCEL_REV_ID_ADDR)
        mag = self._read_byte(BNO055.BNO055_MAG_REV_ID_ADDR)
        gyro = self._read_byte(BNO055.BNO055_GYRO_REV_ID_ADDR)
        bl = self._read_byte(BNO055.BNO055_BL_REV_ID_ADDR)
        sw_lsb = self._read_byte(BNO055.BNO055_SW_REV_ID_LSB_ADDR)
        sw_msb = self._read_byte(BNO055.BNO055_SW_REV_ID_MSB_ADDR)
        sw = ((sw_msb << 8) | sw_lsb) & 0xFFFF
        # Return the results as a tuple of all 5 values.
        return (sw, bl, accel, mag, gyro)

    def set_external_crystal(self, external_crystal):
        """Set if an external crystal is being used by passing True, otherwise
        use the internal oscillator by passing False (the default behavior).
        """
        # Switch to configuration mode.
        self._config_mode()
        # Set the clock bit appropriately in the SYS_TRIGGER register.
        if external_crystal:
            self._write_byte(BNO055.BNO055_SYS_TRIGGER_ADDR, 0x80)
        else:
            self._write_byte(BNO055.BNO055_SYS_TRIGGER_ADDR, 0x00)
        # Go back to normal operation mode.
        self._operation_mode()

    def get_system_status(self, run_self_test=True):
        """Return a tuple with status information.  Three values will be returned:
          - System status register value with the following meaning:
              0 = Idle
              1 = System Error
              2 = Initializing Peripherals
              3 = System Initialization
              4 = Executing Self-Test
              5 = Sensor fusion algorithm running
              6 = System running without fusion algorithms
          - Self test result register value with the following meaning:
              Bit value: 1 = test passed, 0 = test failed
              Bit 0 = Accelerometer self test
              Bit 1 = Magnetometer self test
              Bit 2 = Gyroscope self test
              Bit 3 = MCU self test
              Value of 0x0F = all good!
          - System error register value with the following meaning:
              0 = No error
              1 = Peripheral initialization error
              2 = System initialization error
              3 = Self test result failed
              4 = Register map value out of range
              5 = Register map address out of range
              6 = Register map write error
              7 = BNO low power mode not available for selected operation mode
              8 = Accelerometer power mode not available
              9 = Fusion algorithm configuration error
             10 = Sensor configuration error

        If run_self_test is passed in as False then no self test is performed and
        None will be returned for the self test result.  Note that running a
        self test requires going into config mode which will stop the fusion
        engine from running.
        """
        self_test = None
        if run_self_test:
            # Switch to configuration mode if running self test.
            self._config_mode()
            # Perform a self test.
            sys_trigger = self._read_byte(BNO055.BNO055_SYS_TRIGGER_ADDR)
            self._write_byte(BNO055.BNO055_SYS_TRIGGER_ADDR, sys_trigger | 0x1)
            # Wait for self test to finish.
            time.sleep(1.0)
            # Read test result.
            self_test = self._read_byte(BNO055.BNO055_SELFTEST_RESULT_ADDR)
            # Go back to operation mode.
            self._operation_mode()
        # Now read status and error registers.
        status = self._read_byte(BNO055.BNO055_SYS_STAT_ADDR)
        error = self._read_byte(BNO055.BNO055_SYS_ERR_ADDR)
        # Return the results as a tuple of all 3 values.
        return (status, self_test, error)

    def get_calibration_status(self):
        """Read the calibration status of the sensors and return a 4 tuple with
        calibration status as follows:
          - System, 3=fully calibrated, 0=not calibrated
          - Gyroscope, 3=fully calibrated, 0=not calibrated
          - Accelerometer, 3=fully calibrated, 0=not calibrated
          - Magnetometer, 3=fully calibrated, 0=not calibrated
        """
        # Return the calibration status register value.
        cal_status = self._read_byte(BNO055.BNO055_CALIB_STAT_ADDR)
        sys = (cal_status >> 6) & 0x03
        gyro = (cal_status >> 4) & 0x03
        accel = (cal_status >> 2) & 0x03
        mag = cal_status & 0x03
        # Return the results as a tuple of all 3 values.
        return (sys, gyro, accel, mag)

    def get_calibration(self):
        """Return the sensor's calibration data and return it as an array of
        22 bytes. Can be saved and then reloaded with the set_calibration function
        to quickly calibrate from a previously calculated set of calibration data.
        """
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        self._config_mode()
        # Read the 22 bytes of calibration data and convert it to a list (from
        # a bytearray) so it's more easily serialized should the caller want to
        # store it.
        cal_data = list(self._read_bytes(BNO055.ACCEL_OFFSET_X_LSB_ADDR, 22))
        # Go back to normal operation mode.
        self._operation_mode()
        return cal_data

    def set_calibration(self, data):
        """Set the sensor's calibration data using a list of 22 bytes that
        represent the sensor offsets and calibration data.  This data should be
        a value that was previously retrieved with get_calibration (and then
        perhaps persisted to disk or other location until needed again).
        """
        # Check that 22 bytes were passed in with calibration data.
        if data is None or len(data) != 22:
            raise ValueError('Expected a list of 22 bytes for calibration data.')
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        self._config_mode()
        # Set the 22 bytes of calibration data.
        self._write_bytes(BNO055.ACCEL_OFFSET_X_LSB_ADDR, data)
        # Go back to normal operation mode.
        self._operation_mode()

    def get_axis_remap(self):
        """Return a tuple with the axis remap register values.  This will return
        6 values with the following meaning:
          - X axis remap (a value of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z.
                          which indicates that the physical X axis of the chip
                          is remapped to a different axis)
          - Y axis remap (see above)
          - Z axis remap (see above)
          - X axis sign (a value of AXIS_REMAP_POSITIVE or AXIS_REMAP_NEGATIVE
                         which indicates if the X axis values should be positive/
                         normal or negative/inverted.  The default is positive.)
          - Y axis sign (see above)
          - Z axis sign (see above)

        Note that by default the axis orientation of the BNO chip looks like
        the following (taken from section 3.4, page 24 of the datasheet).  Notice
        the dot in the corner that corresponds to the dot on the BNO chip:

                           | Z axis
                           |
                           |   / X axis
                       ____|__/____
          Y axis     / *   | /    /|
          _________ /______|/    //
                   /___________ //
                  |____________|/
        """
        # Get the axis remap register value.
        map_config = self._read_byte(BNO055.BNO055_AXIS_MAP_CONFIG_ADDR)
        z = (map_config >> 4) & 0x03
        y = (map_config >> 2) & 0x03
        x = map_config & 0x03
        # Get the axis remap sign register value.
        sign_config = self._read_byte(BNO055.BNO055_AXIS_MAP_SIGN_ADDR)
        x_sign = (sign_config >> 2) & 0x01
        y_sign = (sign_config >> 1) & 0x01
        z_sign = sign_config & 0x01
        # Return the results as a tuple of all 3 values.
        return (x, y, z, x_sign, y_sign, z_sign)

    def set_axis_remap(self, x, y, z,
                       x_sign=AXIS_REMAP_POSITIVE, y_sign=AXIS_REMAP_POSITIVE,
                       z_sign=AXIS_REMAP_POSITIVE):
        """Set axis remap for each axis.  The x, y, z parameter values should
        be set to one of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z and will
        change the BNO's axis to represent another axis.  Note that two axises
        cannot be mapped to the same axis, so the x, y, z params should be a
        unique combination of AXIS_REMAP_X, AXIS_REMAP_Y, AXIS_REMAP_Z values.

        The x_sign, y_sign, z_sign values represent if the axis should be positive
        or negative (inverted).

        See the get_axis_remap documentation for information on the orientation
        of the axises on the chip, and consult section 3.4 of the datasheet.
        """
        # Switch to configuration mode.
        self._config_mode()
        # Set the axis remap register value.
        map_config = 0x00
        map_config |= (z & 0x03) << 4
        map_config |= (y & 0x03) << 2
        map_config |= x & 0x03
        self._write_byte(BNO055.BNO055_AXIS_MAP_CONFIG_ADDR, map_config)
        # Set the axis remap sign register value.
        sign_config = 0x00
        sign_config |= (x_sign & 0x01) << 2
        sign_config |= (y_sign & 0x01) << 1
        sign_config |= z_sign & 0x01
        self._write_byte(BNO055.BNO055_AXIS_MAP_SIGN_ADDR, sign_config)
        # Go back to normal operation mode.
        self._operation_mode()
        
        #ベクトル
    def _read_vector(self, address, count=3):
        # Read count number of 16-bit signed values starting from the provided
        # address. Returns a tuple of the values that were read.
        data = self._read_bytes(address, count*2)
        result = [0]*count
        for i in range(count):
            result[i] = ((data[i*2+1] << 8) | data[i*2]) & 0xFFFF
            if result[i] > 32767:
                result[i] -= 65536
        return result

        #オイラー角
    def read_euler(self):
        """Return the current absolute orientation as a tuple of heading, roll,
        and pitch euler angles in degrees.
        """
        heading, roll, pitch = self._read_vector(BNO055.BNO055_EULER_H_LSB_ADDR)
        return (heading/16.0, roll/16.0, pitch/16.0)

        #地磁気
    def read_magnetometer(self):
        """Return the current magnetometer reading as a tuple of X, Y, Z values
        in micro-Teslas.
        """
        x, y, z = self._read_vector(BNO055.BNO055_MAG_DATA_X_LSB_ADDR)
        return (x/16.0, y/16.0, z/16.0)
    
    def read_magnetometer_x(self):
        """Return the current magnetometer reading as a tuple of X, Y, Z values
        in micro-Teslas.
        """
        x, y, z = self._read_vector(BNO055.BNO055_MAG_DATA_X_LSB_ADDR)
        return (x/16.0)
    
    def read_magnetometer_y(self):
        """Return the current magnetometer reading as a tuple of X, Y, Z values
        in micro-Teslas.
        """
        x, y, z = self._read_vector(BNO055.BNO055_MAG_DATA_X_LSB_ADDR)
        return (y/16.0)
    
    def read_magnetometer_z(self):
        """Return the current magnetometer reading as a tuple of X, Y, Z values
        in micro-Teslas.
        """
        x, y, z = self._read_vector(BNO055.BNO055_MAG_DATA_X_LSB_ADDR)
        return (z/16.0)
    
    def read_magnetometer_list(self):
        """Return the current magnetometer reading as a tuple of X, Y, Z values
        in micro-Teslas.
        """
        x, y, z = self._read_vector(BNO055.BNO055_MAG_DATA_X_LSB_ADDR)
        return [x/16.0, y/16.0, z/16.0]
    
    #角加速度
    def read_gyroscope(self):
        """Return the current gyroscope (angular velocity) reading as a tuple of
        X, Y, Z values in degrees per second.
        """
        x, y, z = self._read_vector(BNO055.BNO055_GYRO_DATA_X_LSB_ADDR)
        return (x/900.0, y/900.0, z/900.0)
    
    def read_gyroscope_x(self):
        """Return the current gyroscope (angular velocity) reading as a tuple of
        X, Y, Z values in degrees per second.
        """
        x, y, z = self._read_vector(BNO055.BNO055_GYRO_DATA_X_LSB_ADDR)
        return (x/900.0)
    
    def read_gyroscope_y(self):
        """Return the current gyroscope (angular velocity) reading as a tuple of
        X, Y, Z values in degrees per second.
        """
        x, y, z = self._read_vector(BNO055.BNO055_GYRO_DATA_X_LSB_ADDR)
        return (y/900.0)
    
    def read_gyroscope_z(self):
        """Return the current gyroscope (angular velocity) reading as a tuple of
        X, Y, Z values in degrees per second.
        """
        x, y, z = self._read_vector(BNO055.BNO055_GYRO_DATA_X_LSB_ADDR)
        return (z/900.0)
    
    #加速度
    def read_accelerometer(self):
        """Return the current accelerometer reading as a tuple of X, Y, Z values
        in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_ACCEL_DATA_X_LSB_ADDR)
        return (x/100.0, y/100.0, z/100.0)
    
    def read_accelerometer_x(self):
        """Return the current accelerometer reading as a tuple of X, Y, Z values
        in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_ACCEL_DATA_X_LSB_ADDR)
        return (x/100.0)
    
    def read_accelerometer_y(self):
        """Return the current accelerometer reading as a tuple of X, Y, Z values
        in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_ACCEL_DATA_X_LSB_ADDR)
        return ( y/100.0)
    
    def read_accelerometer_z(self):
        """Return the current accelerometer reading as a tuple of X, Y, Z values
        in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_ACCEL_DATA_X_LSB_ADDR)
        return (z/100.0)

        #線形加速度
    
    def read_linear_acceleration(self):
        """Return the current linear acceleration (acceleration from movement,
        not from gravity) reading as a tuple of X, Y, Z values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR)
        return (x/100.0, y/100.0, z/100.0)
    
    def read_linear_acceleration_list(self):
        """Return the current linear acceleration (acceleration from movement,
        not from gravity) reading as a tuple of X, Y, Z values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR)
        return [x/100.0, y/100.0, z/100.0]
    
    def read_linear_acceleration_x(self):
        """Return the current linear acceleration (acceleration from movement,
        not from gravity) reading as a tuple of X, Y, Z values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR)
        return (x/100.0)
    
    def read_linear_acceleration_y(self):
        """Return the current linear acceleration (acceleration from movement,
        not from gravity) reading as a tuple of X, Y, Z values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR)
        return (y/100.0)
    
    def read_linear_acceleration_z(self):
        """Return the current linear acceleration (acceleration from movement,
        not from gravity) reading as a tuple of X, Y, Z values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR)
        return (z/100.0)

        #重力
    def read_gravity(self):
        """Return the current gravity acceleration reading as a tuple of X, Y, Z
        values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_GRAVITY_DATA_X_LSB_ADDR)
        return (x/100.0, y/100.0, z/100.0)
    
    def read_gravity_x(self):
        """Return the current gravity acceleration reading as a tuple of X, Y, Z
        values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_GRAVITY_DATA_X_LSB_ADDR)
        return (x/100.0)
    
    def read_gravity_y(self):
        """Return the current gravity acceleration reading as a tuple of X, Y, Z
        values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_GRAVITY_DATA_X_LSB_ADDR)
        return (y/100.0)
    
    def read_gravity_z(self):
        """Return the current gravity acceleration reading as a tuple of X, Y, Z
        values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055.BNO055_GRAVITY_DATA_X_LSB_ADDR)
        return (z/100.0)

        #クォータニオン
    def read_quaternion(self):
        """Return the current orientation as a tuple of X, Y, Z, W quaternion
        values.
        """
        w, x, y, z = self._read_vector(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 4)
        # Scale values, see 3.6.5.5 in the datasheet.
        scale = (1.0 / (1<<14))
        return (x*scale, y*scale, z*scale, w*scale)
    
    def read_quaternion_x(self):
        """Return the current orientation as a tuple of X, Y, Z, W quaternion
        values.
        """
        w, x, y, z = self._read_vector(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 4)
        # Scale values, see 3.6.5.5 in the datasheet.
        scale = (1.0 / (1<<14))
        return (x*scale)
    
    def read_quaternion_y(self):
        """Return the current orientation as a tuple of X, Y, Z, W quaternion
        values.
        """
        w, x, y, z = self._read_vector(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 4)
        # Scale values, see 3.6.5.5 in the datasheet.
        scale = (1.0 / (1<<14))
        return (y*scale)
    
    def read_quaternion_z(self):
        """Return the current orientation as a tuple of X, Y, Z, W quaternion
        values.
        """
        w, x, y, z = self._read_vector(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 4)
        # Scale values, see 3.6.5.5 in the datasheet.
        scale = (1.0 / (1<<14))
        return (z*scale)
        
        #クォータニオン角度
    def read_quaternion_w(self):
        """Return the current orientation as a tuple of X, Y, Z, W quaternion
        values.
        """
        w, x, y, z = self._read_vector(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 4)
        # Scale values, see 3.6.5.5 in the datasheet.
        scale = (1.0 / (1<<14))
        return (w*scale)

        #温度
    def read_temp(self):
        """Return the current temperature in Celsius."""
        return self._read_signed_byte(BNO055.BNO055_TEMP_ADDR)
    
    def get_myangle(self) :
        #!データが真円になるように補正する必要あり
        Px = float(magnet_calibrate_list[0])
        Py = float(magnet_calibrate_list[1])
        #Px =-60                            
        #Py =250
        magnetlist = self.read_magnetometer_list()
        #magnetlist = [100,0]             #ダミーデータ
        magnet_x  = float(magnetlist[0])
        magnet_y  = float(magnetlist[1])
        
        x = magnet_x - Px
        y = magnet_y - Py
        
        myangle = (0 - math.degrees(math.atan2(y,x))) % 360
        
        return myangle
    
    #機体の方角を取得(GPS)
    def get_myangle2(past_gps, now_gps) :
        past_la  = float(past_gps[0])
        past_lo  = float(past_gps[1])
        now_la  = float(now_gps[0])
        now_lo  = float(now_gps[1])
    
        la = now_la - past_la
        lo = now_lo - past_lo
    
        myangle = (90 - math.degrees(math.atan2(la,lo))) % 360
    
        return myangle

#MPU9250
bus_number = 1
bus = smbus.SMBus(bus_number)
Accel_Gyro_sensor_device_addr = 0x68 #0x61かもしれない(確認すること)
#加速度センサの値を取得するためにアドレスを入手する
Magnet_sensor_devidce_addr = 0x0C 
#   0x00 0x02 にアクセス

#3軸加速度データを取得
def Get_Accel_status():
    addr = 0x68
    bus_number = 1
    bus = smbus.SMBus(bus_number)
    global pro_count
    pro_count = "GAS"
    scale = 2
    Address_map = [[0x3B , 0x3C] , [0x3D , 0x3E] , [0x3F , 0x40]]
    status = [0,0,0]
    k = 0
    for i in Address_map:
        for j in i:
            n =  bus.read_byte_data(addr,j)
            status[k] = (status[k] << 8) | n
        if status[k] & 0x8000:
            status[k] = -1 * ((status[k] ^ 0xFFFF) + 1)
        status[k] /= float(0x8000/scale)
        k += 1
    #戻り値はリスト形式で[x,y,z]
    return status
#3軸各加速度データを取得
def Get_Gyro_status():
    addr = 0x68
    bus_number = 1
    bus = smbus.SMBus(bus_number)
    global pro_count
    scale = 250
    pro_count = "GGS"
    Address_map = [[0x43 , 0x44] , [0x45 , 0x46] , [0x47 , 0x48]]
    status = [0,0,0]
    k = 0
    for i in Address_map:
        for j in i:
            n =  bus.read_byte_data(addr,j)
            status[k] = (status[k] << 8) | n
        if status[k] & 0x8000:
            status[k] = -1 * ((status[k] ^ 0xFFFF) + 1)
        status[k] /= float(0x8000/scale)
        k += 1
    #戻り値はリスト形式で[x,y,z]
    return status
    
#3軸地磁気データを取得
def Get_Magnet_status():
    addr = 0x0C
    bus_number = 1
    bus = smbus.SMBus(bus_number)
    global pro_count
    pro_count = "GMS"
    scale = 4912#MAX:4912マイクロテスラ
    Address_map= [[0x04 , 0x03] , [0x06 , 0x05] , [0x08 , 0x07]]
    status = [0,0,0]
    k = 0
    for i in Address_map:
        for j in i:
            n  =  bus.read_byte_data(addr,j)
            status[k] = (int(status[k]) << 8) | n
            status[k] = int(status[k])
            #print(status[k])
        if int(status[k]) & int(0x8000):
            status[k] = -1 * ((status[k] ^ 0xFFFF) + 1)
        status[k] /= float(8000/scale)
        #print(status[k])
        k += 1
    ST2 = 0x09 #ST2レジスタ、データ読み出し完了時に読み出し
    #(読み出し中はデータ破損防止のためST2が読まれるまでデータ更新が停止)
    bus.read_byte_data(addr,ST2)
    #戻り値はリスト形式で[x,y,z]
    return status

def Set_Magnet_Cofigdata():
    bus_number = 1
    bus = smbus.SMBus(bus_number)
    addr = 0x0C
    global pro_count
    pro_count = "SMC"
    bus.write_byte_data(0x68,0x37,0x02)
    Address_map= [0x0A]
    config_status = [0b00010110]
    #mode1 0010 mode2 0100 14bit [4] = 0 16bit [4] = 1
    for i in Address_map:
        bus.write_byte_data(addr,i,config_status[Address_map.index(i)])
    return 0

def write_bno055_register(register,value):
    bus_number = 1 #i2cパス番号
    device_address = 0x28 #BNO055のI2Cアドレス
    bus = smbus.SMBus(bus_number)
    #registerに値を書き込む
    bus.read_byte_data(device_address,register,value)
    time.sleep(0.01)
    
def Set_AccelGyro_Configdata():
    bus_number = 1
    bus = smbus.SMBus(bus_number)
    addr = 0x68
    global pro_count
    pro_count = "SAGC"
#    print(bus.write_byte_data(0x68,107,0b00000000))
    Address_map= [0x1A , 0x1B , 0x1C , 0x1D , 0x68 , 0x6B , 0x37]
    config_status = [0b00000001 , 0b00000000 ,0b00000000 , 0b00000000 , 0b00000111 , 0b00000000 , 0b00000010]
    #mode1 0010 mode2 0100 14bit [4] = 0 16bit [4] = 1
    #0b00000000 , 0b00001011 ,0b00001000 , 0b00000000 , 0b00000111
    for i in Address_map:
        j = 0
        n = config_status[j]
        bus.write_byte_data(addr,i,n)
        j += 1
        time.sleep(0.05)
    return 0

#MPUのセットアップ
def MPU_setup():
    write_bno055_register(0x3D,0x00)

def cal_lib():
    max_mg = [0 , 0 , 0]
    min_mg = [0 , 0 , 0]
    k = 100
    while(k):
        n = Get_Magnet_status()
        k = k - 1
        time.sleep(0.05)
        for k in range(3):
            if max_mg[k] < n[k]:
                max_mg[k] = n[k]
            if min_mg[k] > n[k]:
                min_mg[k] = n[k]
            offset = [0 , 0 , 0]
        for l in range(3):
            offset[l] = max_mg[l] / 2 + min_mg[l] / 2
        print(offset)

def tan_calc():
    Magnet_status = Get_Magnet_status()
    n = Magnet_status
    offset = [-7.875, -44.25, 28.2]
    for l in range(3):
        Magnet_status[l] = Magnet_status[l] - offset[l]
        #            print(math.sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]))
        for i in range(3):
            for j in range(3):
                if i != j and i < j:
                    RT_rad = np.arctan(Magnet_status[i]/(Magnet_status[j] + 0.01))
                    print(i,j,RT_rad)
                    RT_rad = np.arctan(Magnet_status[j]/(Magnet_status[i] + 0.01))
                    print(j,i,RT_rad)

def s_atamawarui_hannbetuki(data):
    # N , E , S , W
    hougaku = [0 , 1.57 , 3.14 , 4.71]
    x_sta = [20 , 40 , 20,  -10]
    y_sta = [20 , -10 , -30 , -10]
    z_sta = [15 , 20 , 20 , 15]
    score = [0 , 0 , 0 , 0]
    for k in range(4):
        xc = (data[0] - x_sta[k]) **2
        yc = (data[1] - y_sta[k]) **2
        zc = (data[2] - z_sta[k]) **2
        score[k] = xc + yc + zc
#    print(score)
    out_d = hougaku[score.index(min(score))]
    return out_d

#フォトレジスタ
spi = spidev.SpiDev()

#フォトレジスタと抵抗の合成抵抗の電圧を取得(日中は2.5～3.3Vを返す暗くなると0.5～0V程度を返す)
def read_voltage():
    spi.open(0, 0) # 0：SPI0、0：CE0
    spi.max_speed_hz = 1000000 # 1MHz SPIのバージョンアップによりこの指定をしないと動かない
    dout = spi.xfer2([((0b1000+CHN)>>2)+0b100,((0b1000+CHN)&0b0011)<<6,0]) # Din(RasPi→MCP3208）を指定
    bit12 = ((dout[1]&0b1111) << 8) + dout[2] # Dout（MCP3208→RasPi）から12ビットを取り出す
    volts = round((bit12 * V_REF) / float(4095),4)  # 取得した値を電圧に変換する（12bitなので4095で割る）
    spi.close()
    return volts # 電圧を返す

def stop_spi() :
    spi.close()

#カメラ  
#引数は整数で撮影した画像を保存する時の名前の一部になる  
def ca_main(camera_count):
    i = camera_count
    pic_name ="redcorn"+str(i)+ ".jpg"
    # カメラ初期化
    camera  = picamera.PiCamera()
    # 解像度の設定
    camera.resolution = (640, 480)
    # 撮影の準備
    camera.start_preview()
    # 準備している間、少し待機する
    time.sleep(0)
    # 撮影して指定したファイル名で保存する
    camera.capture(pic_name)
    camera.stop_preview()
    camera.close()

# GPIOにアクセスするためのインスタンスを作成
pi = pigpio.pi()                             

def moter_setup():
    # GPIO pin を出力設定
    pi.set_mode(LEFT_TOP_M1, pigpio.OUTPUT)     
    pi.set_mode(LEFT_TOP_M2, pigpio.OUTPUT)
    pi.set_mode(RIGHT_TOP_M1, pigpio.OUTPUT)
    pi.set_mode(RIGHT_TOP_M2, pigpio.OUTPUT)
    pi.set_mode(CAREER_CUT, pigpio.OUTPUT)
    #GPIOの周波数を設定
    pi.set_PWM_frequency(LEFT_TOP_M1, 50)
    pi.set_PWM_frequency(LEFT_TOP_M2, 50)
    pi.set_PWM_frequency(RIGHT_TOP_M1, 50)
    pi.set_PWM_frequency(RIGHT_TOP_M2, 50)
    #GPIOの範囲を設定
    pi.set_PWM_range(LEFT_TOP_M1, 100)
    pi.set_PWM_range(LEFT_TOP_M2, 100)
    pi.set_PWM_range(RIGHT_TOP_M1, 100)
    pi.set_PWM_range(RIGHT_TOP_M2, 100)
    #電熱線設定
    pi.write(CAREER_CUT , 0)

def stop():
    pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    pi.write(CAREER_CUT , 0)

def gostop(start, max = 100.0, t = 1.0) :
    s = max / t / 10.0
    for duty in drange2(max, 0.0, s):
        pi.set_PWM_dutycycle(LEFT_TOP_M1, duty)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, duty)
        time.sleep(0.1)

def drange1(begin, end, step):
    n = begin
    while n+step < end:
        yield n
        n += step

def drange2(begin, end, step):
    n = begin
    while n+step > end:
        yield n
        n -= step        

def step(pin, max = 100.0, t = 1.0):
    s = max / t / 10.0
    for duty in drange1(0.0, max, s):
        pi.set_PWM_dutycycle(pin, duty)
        time.sleep(0.1)

def go(start, max = 100.0, t = 1.0):
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    if start:
        s = max / t / 10.0
        for duty in drange1(0.0, max, s):
            pi.set_PWM_dutycycle(LEFT_TOP_M1, duty)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, duty)
            time.sleep(0.1)
    else:
        pi.set_PWM_dutycycle(LEFT_TOP_M1, max)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, max)

def go2(start, rmax =55, lmax=100 , t = 1.0):
    #lmax,rmaxの設定をいじることで左右のモーターの回転数のバランスを調節する
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    pi.set_PWM_dutycycle(LEFT_TOP_M1, lmax)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, rmax)

def back(start, max = 100.0, t = 1.0):
    pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
    if start:
        s = max / t / 10.0
        for duty in drange1(0.0, max, s):
            pi.set_PWM_dutycycle(LEFT_TOP_M2, duty)
            pi.set_PWM_dutycycle(RIGHT_TOP_M2, duty)
            time.sleep(0.1)
    else:
        pi.set_PWM_dutycycle(LEFT_TOP_M2, max)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, max)

def left(start, max = 100.0, t = 1.0, rate = 1.0):
    if rate >= 0:
        r = max
        l = rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        #if start:
        #    s = max / t / 10.0
        #    for duty in drange1(0.0, max, s):
        #        if duty <= l:
        #            pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        #        if duty <= r:
        #            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
        #else:
        pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
    else:
        r = max
        l = - rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        #if start:
        #    s = max / t / 10.0
        #    for duty in drange1(0.0, max, s):
        #        if duty <= l:
        #            pi.set_PWM_dutycycle(LEFT_TOP_M2, l)
        #        if duty <= r:
        #            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
        #else:
        pi.set_PWM_dutycycle(LEFT_TOP_M2, l)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)

def right(start, max = 100.0, t = 1.0, rate = 1.0):
    if rate >= 0:
        l = max
        r = rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        #if start:
        #    s = max / t / 10.0
        #    for duty in drange1(0.0, max, s):
        #        if duty <= l:
        #            pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        #        if duty <= r:
        #            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
        #else:
        pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, r)
    else:
        l = max
        r = - rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
        #if start:
        #    s = max / t / 10.0
        #    for duty in drange1(0.0, max, s):
        #        if duty <= l:
        #            pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        #        if duty <= r:
        #            pi.set_PWM_dutycycle(RIGHT_TOP_M2, r)
        #else:
        pi.set_PWM_dutycycle(LEFT_TOP_M1, l)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, r)

#脱出用の関数
def escape() :
        left(0,75,rate = -1)
        time.sleep(2.0)
        stop()
        right(0,75,rate = -1)
        time.sleep(2.0)
        stop()
        go2(0)
        time.sleep(1)
        stop()
        right(0,75,rate = -1)
        time.sleep(0.35)
        stop()
        go2(0)
        time.sleep(10)
        stop()

#電熱線の燃焼を行う関数
#引数は燃焼時間デフォルトで1.5s
def career_cat(t=4) :
    career_time = t
    print("点火")
    pi.write(CAREER_CUT , 1 )
    time.sleep(career_time)
    pi.write(CAREER_CUT , 0 )
    time.sleep(0.1)

#色認識(産廃)
def red_search(camera_count) :
    i = camera_count
    pic_name = "redcorn"+str(i)+".jpg"
    pic_name_out = "redcorn"+str(i)+"out.jpg"
    mask_pic = "redcorn"+str(i)+"mask.jpg"
    
    #画像読み込み
    img = cv2.imread(pic_name)
    #画像を180°回転
    img = cv2.rotate(img, cv2.ROTATE_180)
    #cv2.imwrite('redcorn777_rotate_180.jpg', img)
    #画像をHSV化
    img_HSV1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    #cv2.imwrite('redcorn_HSV.jpg', img_HSV)
    #画像の平滑化
    img_HSV2 = cv2.GaussianBlur(img_HSV1, (9, 9), 3)
    #cv2.imwrite('redcorn_HSV2.jpg', img_HSV)
    
    #BGRで画像を分割
    img_B, img_G, img_R = cv2.split(img_HSV2)
    #画像確認用
    #cv2.imwrite('redcorn_H.jpg', img_H)
    #cv2.imwrite('redcorn_S.jpg', img_S)
    #cv2.imwrite('redcorn_V.jpg', img_V)
    
    #画像の2値(赤とそれ以外に分割した画像を作成)
    img_thre, img_redcorn_gray = cv2.threshold(img_G, 175, 255, cv2.THRESH_BINARY)
    cv2.imwrite(mask_pic, img_redcorn_gray)
    
    #輪郭を検出
    #labels, contours, hierarchy = cv2.findContours(img_redcorn_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE) #バージョンによってはこちらを使用する
    contours, hierarchy = cv2.findContours(img_redcorn_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for i in range(0, len(contours)):
        if len(contours[i]) > 0:

            #小さなオブジェクトを切り取る
            if cv2.contourArea(contours[i]) < 300:
                continue

            #画像に境界を線で出力
            cv2.polylines(img, contours[i], True, (0, 0, 0), 5)
    
    #画像を出力
    cv2.imwrite(pic_name_out, img)
    
    #モーメントを取得
    M = cv2.moments(img_redcorn_gray, False)
    #赤色部分の重心の座標を計算
    x,y= int(M["m10"]/(M["m00"]+1)) , int(M["m01"]/(M["m00"]+1))
    #重心座標を円で出力
    cv2.circle(img, (x,y), 20, 100, 2, 4)
    cv2.drawContours(img, contours, -1, color=(0, 0, 0), thickness=5)
    
    #画像を出力
    cv2.imwrite('redcorn_out2.jpg', img)
    
    #座標を出力
    out = [x , y]
    print(out)

    return out

#色認識（及川式）こっちを使用する
def red_search2(camera_count):
    for i in range(1):
        c = camera_count
        x = 0
        y = 0
        pic_name="redcorn"+str(c)+ ".jpg"
        pic_name_out="redcorn"+str(c)+ "out.jpg"
        gray_pic="redcorn"+str(c)+ "gray.jpg"
        #画像読み込み
        img = cv2.imread(pic_name,cv2.IMREAD_COLOR)
        #画像を180°回転
        img = cv2.rotate(img, cv2.ROTATE_180)
        #画像の平滑化
        img = cv2.GaussianBlur(img, (9, 9), 3)
        height, width = img.shape[:2]
        #画像のHSV化
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        img_redcorn=np.zeros((height,width,3),np.uint8)
        #画像の2値化
        img_redcorn[(h <=255) & (h > 190) + (h <30) & (h >= 0) & (s > 80) & (v > 80)] = 255 #! (h <=255) & (h > 190) + (h <30) & (h >= 0) & (s > 80) & (v > 80)の値を変えることによって微妙な赤色の変化に対応できます！
        #画像を出力
        cv2.imwrite(gray_pic,np.array(img_redcorn))
        img_gray = cv2.imread(gray_pic,cv2.IMREAD_GRAYSCALE)
        #モーメントを取得
        M = cv2.moments(img_gray, False)
        #輪郭抽出
        contours, hierarchy = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #図心を出す
        x,y= int(M["m10"]/(M["m00"]+1)) , int(M["m01"]/(M["m00"]+1))
        #Zerodivision_kaihi
        #図心を画像に記載する
        cv2.circle(img, (x,y), 20, 100, 2, 4)
        cv2.drawContours(img, contours, -1, color=(0, 0, 0), thickness=5)
        cv2.imwrite(pic_name_out,np.array(img))
        out = [x , y]
        print(out)
        return out

#log取得
def log_setup() :
    logname = ["dt_now", ",", "gps_x", ",", "gps_y" , ",","altitude", ",", "photvolt"
               , "action", ",", "my_angle", ",", "goal_angle", ",", "goal_destance", "\n"]
    f0 = open('logdete.txt', 'a', encoding='UTF-8')
    f0.writelines(logname)
    f0.close()
    print("set_log")
    
    return 0

def write_log(seqence = "a",gps="0,0",altitude = 0 ,photvolt = 0 ,diredtion = 0 ,my_angle = 0,goal_angle = 0,goal_destance = 0) :
    dt_now = str(datetime.datetime.now())
    log_list = [",", str(seqence), ",", str(gps), ",", str(altitude), ",", str(photvolt), ",",
                str(diredtion), ",", str(my_angle), ",", str(goal_angle), ",", str(goal_destance),",", str(seqence)]
    
    with open("logdete.txt" , "a") as lf :
        lf.writelines(dt_now)
        lf.writelines(log_list[1:len(log_list)-1:1])
        lf.writelines("\n")
        lf.close
    print("write_log")
    return 0

#シーケンス取得
def get_sequence_count() :
    with open("sequence_count.txt" , "r") as sf :
        sequence_count =sf.read()
        sf.close
    print("get_seqence")
    return int(sequence_count)

#シーケンス保存
def write_sequence_count(sequence_count) :
    sc = str(sequence_count)
    with open("sequence_count.txt" , "w") as sf :
        sequence_count = sf.writelines(sc)
        sf.close
    return 0

#通信開始用フラグ保存
def write_im920flag(flag) :
    f = str(flag)
    with open("im_flag.txt" , "w") as imf :
        flag = imf.writelines(f)
        imf.close
    return 0

def get_preset_pres() :
    with open("preset_pres.txt" , "r") as pf :
        preset_pres =pf.read()
        pf.close
    print("get_preset_pres")
    return float(preset_pres)

def get_preset_magnet() :
    with open("magnet_calibrate.txt" , "r") as mf :
        magnet_calibrate =mf.read()
        mf.close
    print("get_preset_pres")
    return magnet_calibrate

#GPS取得
def get_gps() :
    read_rgps()  #!赤GPSを使用する場合はこちらを使用する
    #read_gps()  #!緑GPSを使用する場合はこちらを使用する
    with open("gps_realtime.txt", "r") as tf:
        gps = tf.read()
        tf.close
    #gps = [20.425540,136.081114] #ダミーデータ
    return gps

#機体の現在位置を取得
def get_gps_realtime() :
    read_rgps()
    #read_gps()
    with open("gps_realtime.txt", "r") as tf:
        gps_realtime = tf.read().split(',')
        tf.close
    #gps_realtime = [20.425540,136.081114]
    return gps_realtime

#ゴールの座標を取得
def get_gps_goal() :
    with open("gps_goalpoint.txt", "r") as tf:
        gps_goal = tf.read().split(',')
        tf.close
    #gps_goal =[40.14265945,139.9875916]#2023/8/16
    return gps_goal



#ヒュペニの法則 使用設定値
GPS_A = 6378137.0
E2 = 0.00669437999019758
A1E2 = 633439.32729246

#ゴールまでの距離を取得
def get_goal_distance(gps, goal):
	dy = math.radians(float(gps[0]) - float(goal[0]))
	dx = math.radians(float(gps[1]) - float(goal[1]))
	uy = math.radians((float(gps[0]) + float(goal[0])) / 2.0)
	W = math.sqrt(1.0 - E2 * math.sin(uy) ** 2.0)
	N = GPS_A / W
	M = A1E2 / W ** 3
	
	gps_distance = math.sqrt((dy * M) ** 2 + (dx * N * math.cos(uy)) ** 2)
	
	return gps_distance

# ゴールの方角を取得
def get_goal_angle(gps, goal):
    gps_la  = float(gps[0])
    gps_lo  = float(gps[1])
    goal_la  = float(goal[0])
    goal_lo  = float(goal[1])
    
    la = goal_la - gps_la
    lo = goal_lo - gps_lo

    goal_angle = (90 - math.degrees(math.atan2(la,lo))) % 360
    
    return goal_angle

bno = BNO055()

#方角テスト
def test_angle() :
    while True :
        gps = get_gps_realtime()
        goal = get_gps_goal()
        #gps =[25,150]
        #goal =[25,140]
        ga = get_goal_angle(gps, goal)
        ma = bno.get_myangle()
        ma2= bno.get_myangle2()
        da = ga - ma
        da2 =ga - ma2
        
        print("ゴール方角" + "  " + str(ga) )
        print("   MPU    |   GPS      ")
        print( "機体の方角" + "  " + str(ma) +" | "+"機体の方角" + "  " + str(ma2))
        print("方角差1" + "      " + str(da) +" | " +"方角差2" + "      " + str(da2))
        
        time.sleep(1.5)
    return 0

#モーター駆動時間設定
def get_moter_time(Angle) :
    x =  cycle_time
    moter_time = (x/360) * math.fabs(Angle) 
    return moter_time

#モーター駆動時間テスト
def test_moter_time(angle = 90) :
    t = get_moter_time(angle)
    print("駆動時間"+"  "+str(t))
    
    return 0

#裏返り検知
def rivers() :
    return 0 #未製作

#メイン処理
#使用する前に全ての関数について調べどのような役割があるのか確認すること
def main() :
    #セットアップ
    log_setup()                                   
    moter_setup()                                 
    MPU_setup()                                  
    BME_setup()                                  
    get_calib_param()                            
    goal_point = get_gps_goal()
    preset_pres = get_preset_pres()                  
    sequence_count = -1                           
    sequence_count = get_sequence_count()        
    write_log(sequence_count,diredtion="set_up") 
    print("set_up_OK")                    

    #暗くなったらsequence0へ
    while True :
        print("check1")
        photoresistor_voltage = float(read_voltage())
        print(photoresistor_voltage)
        time.sleep(1)
        if photoresistor_voltage < 1 : # 本番の環境に合わせる
            print("check2")
            break
    sequence_count = 0
    #投下シーケンス
    #フォトレジスタの電圧が2.0V以上になった(CANSATが投下された)ならば無限ループを抜ける処理が行われる
    if sequence_count == 0 :
        
        while True :
            
            photoresistor_voltage = float(read_voltage()) 
            my_point = get_gps_realtime()
            print("check3")
            if photoresistor_voltage > 2.0 :
                print("check4")
                sequence_count = 1
                write_sequence_count(sequence_count) 
                write_im920flag(1) 
                print("release")
                write_log(sequence_count ,gps = get_gps() ,altitude = (preset_pres),photvolt = photoresistor_voltage ,diredtion = get_goal_distance(my_point,goal_point))
                break
        
            else :
                write_log(sequence_count ,gps = get_gps() ,altitude = (preset_pres),photvolt = photoresistor_voltage ,diredtion = get_goal_distance(my_point,goal_point))
                time.sleep(1)  
    
    #第二投下シーケンス(光センサーの使用が不可の場合に使用)
    #上昇と降下を検知して投下判定を行う
    if sequence_count == 0.5 :
        altitude_flag = 0 
        while True :
            if get_altitude(preset_pres) > preset_altiude2 :
                print("rise")
                
                if altitude_flag < 3 :
                    altitude_flag = altitude_flag + 1
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "rise2" ,my_angle = 0 ,goal_angle = get_goal_angle(get_gps_realtime(),goal_point) ,
                              goal_destance = get_goal_distance(get_gps_realtime(),goal_point))
                
                elif altitude_flag == 3 :
                    sequence_count = 1
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "rise3" ,my_angle = 0 ,goal_angle = get_goal_angle(get_gps_realtime(),goal_point) ,
                              goal_destance = get_goal_distance(get_gps_realtime(),goal_point))
                    break
            
            elif get_altitude < preset_altiude2 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "rise1" ,my_angle = 0 ,goal_angle = get_goal_angle(get_gps_realtime(),goal_point) ,
                          goal_destance = get_goal_distance(get_gps_realtime(),goal_point))
            
            time.sleep(0.1)
    
    #降下シーケンス
    if sequence_count == 1 :
        time_flag = 1
        while True :
            
            if get_altitude(preset_pres) < preset_altiude :
                time.sleep(10)
                print("landing_1")
                career_cat(career_time)
                sequence_count = 2
                write_sequence_count(sequence_count)
                break
            
            #競技によって予想着地時間が違うので競技ごとにパラ分離までの時間を設定してください
            elif time_flag > 600 :
                time.sleep(5)
                print("landing_2")
                career_cat(career_time)
                sequence_count = 2
                write_sequence_count(sequence_count)
                break
            
            else :
                time.sleep(1.0)
                time_flag = time_flag + 1
                print(time_flag)
    
    #脱出シーケンス
    if sequence_count == 2 :  
        #回収モジュールから脱出
        escape()
        career_cat(career_time)
        sequence_count = 3
        my_point  = get_gps_realtime()
        write_sequence_count(sequence_count)
        write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                  diredtion = "escape" ,my_angle = 0 ,goal_angle = get_goal_angle(my_point,goal_point) ,
                  goal_destance = get_goal_distance(my_point,goal_point))
        print("脱出")
    
    #遠距離誘導シーケンス
    #複雑なのでちゃんとフローチャートを確認すること
    if sequence_count == 3 : 
        stack_count = 0
        past_point  = get_gps_realtime()
        while True :
            time.sleep(1)
            #機体の位置を取得
            my_point = get_gps_realtime()
            
            if past_point != my_point :
                stack_count = 0 
                #機体の方位を取得
                #my_angle = bno.get_myangle2(past_point, my_point) #!GPS誘導
                my_angle = bno.get_myangle()                       #!地磁気誘導
                print("機体の方角----------|" + str(my_angle) )
                #ゴールまでの方位を計算
                goal_angle = get_goal_angle(my_point,goal_point)
                print("ゴールの方角----------|" + str(goal_angle) )
                #ゴールと機体の角度差を計算
                Angle =   goal_angle - my_angle 
                print("角度差----------|" + str(Angle) )
                #角度に応じてモーター駆動
                if math.fabs(Angle) < 20 or math.fabs(Angle) > 340 :
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "go" ,my_angle = bno.get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                              goal_destance = get_goal_distance(my_point,goal_point))
                    go2(0)
                    print("go")
                    time.sleep(5.0)
                    stop()
                    
            
                elif -340 <= Angle <= -180 or 20 <= Angle <= 180 :
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "left" ,my_angle = bno.get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                              goal_destance = get_goal_distance(my_point,goal_point))
                    left_time = get_moter_time(Angle)
                    left(0,75,rate = -1)
                    time.sleep(left_time)
                    #time.sleep(0.2)
                    print("left"+str(Angle))
                    stop()
                    my_angle = bno.get_myangle()
                    #go2(0)
                    #time.sleep(5.0)
                    #stop()
                
                elif -180 <= Angle <= -20 or 180 <= Angle <= 340 :
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "right" ,my_angle = bno.get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                              goal_destance = get_goal_distance(my_point,goal_point))
                    right_time = get_moter_time(Angle)
                    right(0,75,rate = -1)
                    time.sleep(right_time)
                    #time.sleep(0.2)
                    print("right"+str(Angle))
                    stop()
                    my_angle = bno.get_myangle()
                    #go2(0)
                    #time.sleep(5.0)
                    #stop()
                    
                #ゴールとの距離を取得
                goal_distance = get_goal_distance(my_point,goal_point)
                print("get_goal_distance"+str(goal_distance))
                if goal_distance < preset_goal_distanc :
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = 0 ,my_angle = bno.get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                              goal_destance = get_goal_distance(my_point,goal_point))
                    sequence_count = 4
                    write_sequence_count(sequence_count)
                    break
            
                else :
                    #time.sleep(3.0)
                    past_point = my_point
                    #print("onecycle")
            
            elif stack_count == 3 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "stack" ,my_angle = bno.get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                          goal_destance = get_goal_distance(my_point,goal_point))
                
                print("stack")
                left(0,100,rate = -1)
                time.sleep(get_moter_time(90))
                go2(0)
                time.sleep(1.0)
                right(0,100,rate = -1)
                time.sleep(get_moter_time(90))
                go2(0)
                time.sleep(1.0)
                stop()
                stack_count = 0
            
            else :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0, 
                          diredtion = "stack?" ,my_angle = bno.get_myangle(),goal_angle = get_goal_angle(my_point,goal_point),
                          goal_destance = get_goal_distance(my_point,goal_point))
                go2(0)
                time.sleep(5.0)
                stop()
                stack_count = stack_count + 1 
                print("stack?")
    
    #能代室内用補助誘導
    if sequence_count == 3.5 : 
        goal_angle = 60
        ff = 0
        while True :
            my_angle = bno.get_myangle()
            Angle = my_angle - goal_angle
            if math.fabs(Angle) < 30 or math.fabs(Angle) > 330 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "go" ,my_angle = bno.get_myangle2(past_point, my_point),goal_angle = get_goal_angle(my_point,goal_point),
                          goal_destance = get_goal_distance(my_point,goal_point))
                go2(0)
                time.sleep(2.0)
                stop()
                ff = ff + 1
            
            elif -330 <= Angle <= -180 or 30 <= Angle <= 180 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "left" ,my_angle = bno.get_myangle2(past_point, my_point),goal_angle = get_goal_angle(my_point,goal_point),
                          goal_destance = get_goal_distance(my_point,goal_point))
                left_time = get_moter_time(Angle)
                left(0,75,rate = -1)
                time.sleep(left_time)
                print("left"+str(Angle))
                stop()
                go2(0)
                time.sleep(2.0)
                stop()
                ff = ff + 1
            
            elif -180 <= Angle <= -30 or 180 <= Angle <= 330 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "right" ,my_angle = bno.get_myangle2(past_point, my_point),goal_angle = get_goal_angle(my_point,goal_point),
                          goal_destance = get_goal_distance(my_point,goal_point))
                right_time = get_moter_time(Angle)
                right(0,75,rate = -1)
                time.sleep(right_time)
                print("right"+str(Angle))
                stop()
                go2(0)
                time.sleep(2.0)
                stop()
                ff = ff + 1
            
            if ff == 3 :
                sequence_count == 4
                break


    #近距離誘導シーケンス            
    if sequence_count == 4 :
        f1 = 0
        f2 = 0
        camera_count = 0           #カメラカウント（基本0回）
        move_count   = 0           #カメラ誘導時赤色認識回数（基本0回）
        finish_count = 15           #カメラ誘導時直進回数
        threshold =60              #カメラ誘導時赤色認識した時に直進するかしないかどうか（0~320） 
        while True :
            ca_main(camera_count)
            out_re = red_search2(camera_count)
            camera_count = camera_count + 1
            
            if out_re[0] == 0 and out_re[1] == 0 and f1 == 0 :
                print("探索モード")
                right(0,100,rate=-1)
                time.sleep(0.22)
                stop()
                go2(0)
                time.sleep(1.5)
                stop()
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "red_search" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)
                
            if out_re[0] == 0 and out_re[1] == 0 and f2 == 0 and f1 >= 1 :
                print("認識していないr")
                right(0,100,rate=-1)
                time.sleep(0.20)
                stop()
                f2 = 1
                f1 = f1 + 1
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "lost_r" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)
                
            elif out_re[0] == 0 and out_re[1] == 0 and f2 == 1 and f1 >= 1 :
                print("認識していないl")
                left(0,100,rate=-1)
                time.sleep(0.32)
                stop()
                f2 = 0
                f1 = f1 + 1 
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "lost_l" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)
                
            if f1 == 5 :
                f1 = 0

            if out_re[0] != 0 and out_re[1] != 0 and move_count < finish_count :
                print("赤色認識成功")
            
                if out_re[0] < (320 - threshold) :
                    print("左")
                    left(0,100, rate = -1)
                    time.sleep(0.2)
                    stop()
                    f2 = 0
                    f1 = 1
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "left" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)

                elif out_re[0] > (320 + threshold) :
                    print("右")
                    right(0,100, rate = -1)
                    time.sleep(0.21)
                    stop()
                    f2 = 1
                    f1 = 1
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "right" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)
                
                else :
                    print("直進")
                    go2(0)
                    time.sleep(0.6)
                    f1 = 1
                    move_count = move_count + 1 
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                              ,diredtion = "go" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)

            elif move_count == finish_count :
                print('goal')
                sequence_count = 5
                write_sequence_count(sequence_count)
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 
                          ,diredtion = "goal" ,my_angle = 0 ,goal_angle = 0 ,goal_destance = 0)
                break  

if __name__ == '__main__' :
    while True : 
        try:
            main()
        
        finally :
            stop_spi()
            stop()

