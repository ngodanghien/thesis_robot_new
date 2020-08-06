#!/usr/bin/python
# Author: hiennd
# Date: 06/08/2020
# Test: PASSED

# ROS
import rospy
import roslib
import tf

# Messages
from std_msgs.msg import String, Int8, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Math
import math
from math import sin, cos, pi
import numpy
import numpy as np
import locale
from locale import atof
#
#https://docs.python.org/3/library/struct.html
# < = little-endian  || f = float
#ex: pack('<fff', 1.0, 2.0, 3.0)
#ex: buffer = [0x19, 0x04, 0x9e, 0x3f] || aa= bytearray(buffer)  ||float = unpack('<f', aa)
from struct import *

# OS
import serial
#

class NodeSerialComPort:
    def __init__(self): #ok
        #Node
        rospy.init_node('node_serial_mcu', anonymous=True)
        #Pub
        self.rate = 100  
        self.vel_pub     = rospy.Publisher('vel_pub',    Twist,      queue_size=self.rate)
        self.theta_pub   = rospy.Publisher('theta_pub',  Float32,    queue_size=self.rate)   
        #Sub
        rospy.Subscriber('/cmd_vel', Twist, self.sub_cmd_vel_callback) # def: sub_cmd_vel_callback()

        #Serial
        self.port_name = rospy.get_param('~port','/dev/ttyUSB_MCU')  #ttyUSB0 -> ttyUSB_MCU
        self.baud = int(rospy.get_param('~baud','256000'))
        #ser = serial.Serial(port = port_name, baudrate = baud, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout=1) 
        #note: phai co timeout, neu khong function: ser.read() se blocking
        self.ser = serial.Serial(port = self.port_name, baudrate = self.baud, timeout=1)    #1s

        #data store
        self.Len_Float_rx       = 8 #int(8) # nhan 8 float = 8*4 + 1 = 33 bytes
        self.Len_Float_rx_buff  = 33 #int(self.Len_Float_rx*4 + 1)
        self.data = list([])    #list empty || data.append(newdata) || data.clear() == []
        self.dataFloat = [0]*self.Len_Float_rx #luu data Float

        self.last_time = rospy.Time.now()
        # varible 
        self.vel_pub_msg    = Twist()       #tuong duong '/cmd_vel'
        self.theta_pub_msg  = Float32()
        self.tx_send_ok  = bool(False)      #type bool    |0=None|1=OK=send|
        self.rx_send_ok  = bool(False)      #type bool    |0=None|1=OK=send|
    
    #ref: https://pyserial.readthedocs.io/en/latest/pyserial_api.html
    #ref: https://pyserial.readthedocs.io/en/latest/shortintro.html
    # PC send data to MCU via Serial Tx
    def data_send(self):
        # 02 float need send
        #ex: Serial.write(b"\x01" + struct.pack("<ff", float1, float2))
        self.v = 1.0
        self.w = 2.0
        data = pack('<ff', self.v, self.w) #< = litle-endian
        self.ser.write(data + b'\r')  #PASSED #'\r' = 0x0D = 13
        #rospy.loginfo("[DEBUG][node_serial_mcu][Tx] -- self.ser.write ---, length: = " + str(len(data + b'\r'))) #PASSED
    # PC receive data from MCU via Serial Rx
    def data_receive(self): #Nhan duoc data la gui luon ..... (MCU gui tam ~ 40ms / buff_data)  
        x = self.ser.read(1) #read 1byte, timeout = 1s; return: bytes
        #rospy.loginfo("[DEBUG]" + str(x))
        #rospy.loginfo("[DEBUG]" + str(xx) + str(len(xx)))
        #
        if x == '\r':   #PASSED          
            self.data.append(x)
            try:
                #PASSED
                if len(self.data) == self.Len_Float_rx_buff:
                    #PASSED
                    #rospy.loginfo("[DEBUG] OKOKOKKKOOK" + str(len(self.data)) )
                    for i in range(self.Len_Float_rx): # 0 -> (Len_Float_rx-1)
                        buffer = self.data[i*4:i*4+4]
                        aa= bytearray(buffer)
                        self.dataFloat[i] = unpack('<f', aa)
                        #rospy.loginfo("[DEBUG][node_serial_mcu][Rx]: " +str(i) +": =" +str(self.dataFloat[i])) #PASSED

                    current_time = rospy.Time.now()
                    dt = (current_time - self.last_time).to_sec()   #deta_time
                    self.last_time = current_time                   #update time
                    #rospy.loginfo("[DEBUG][node_serial_mcu][Rx] TimeSample: = " + str(dt) + "s, Freq = " + str(1/dt))
                    #PASSED: 5ms - 200Hz

            except Exception as inst:
                pass
                #rospy.loginfo(type(inst)) 
                rospy.loginfo(inst.args) 
            #reset
            #self.data.clear()
            self.data = []
        else:
            self.data.append(x)   #PASSED

    def pub_def(self):
        self.vel_pub.publish(self.vel_pub_msg)
        self.theta_pub.publish(self.theta_pub_msg)

    def spin(self): #ok
        rospy.loginfo("[ROS][hiennd] Start Node Serial connect to MCU at: " + self.port_name + ", " + str(self.baud) + 'bps')
        rate = rospy.Rate(100000) #100kHz == OK
        
        while not rospy.is_shutdown():
            #0. Receive(rx): cho nhan lien tuc, du data thi pub_def()       
            self.data_receive()      
            #1. Send (tx): chi khi nao, sub_cmd_vel_callback() dc goi thi send()
            if self.tx_send_ok == True:
                self.tx_send_ok = False #reset
                #
                self.data_send()
               
            #2. Publish to ROS Master.
            if self.rx_send_ok == True:  #ok_send
                self.rx_send_ok = False   #reset
                #
                self.pub_def()
            #
            #rate.sleep()    # khong dung, neu dung thif rate = 100kHz
        
    
    def sub_cmd_vel_callback(self, vel_sub_msg):
        # MCU tu dong chuyen ve: wL, wR
        self.v = vel_sub_msg.linear.x        # v robot (m/s)
        self.w = vel_sub_msg.angular.z       # w robot (rad/s)
        self.tx_send_ok = True               # data ready (pre send via serial)

# -------------Main--------------------
def main():
    nodeSerial = NodeSerialComPort()
    nodeSerial.spin()

if __name__ == '__main__':
    main()
