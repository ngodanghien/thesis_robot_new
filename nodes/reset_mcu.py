#!/usr/bin/python
# Author: hiennd
# Date: 15/08/2020
# Test: PASSED ==OK

import serial

from struct import *

cmd = 2.00000001

def data_send():
        # 02 float need send
        #ex: Serial.write(b"\x01" + struct.pack("<ff", float1, float2))
        # == 0x01 = reset IMU (yaw)
        # == 0x02 = reset CPU.
        #cu goi ham nay len la cho init lai IMU
        data = pack('<fff', 0.0, 0.0, cmd) #< = litle-endian, 0.1234: du phong
        ser.write(data + b'\r')  #PASSED #'\r' = 0x0D = 13

ser = serial.Serial(port = '/dev/ttyUSB_MCU', baudrate = 256000, timeout=1)    #1s

data_send()