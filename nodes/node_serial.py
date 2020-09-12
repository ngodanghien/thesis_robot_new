#!/usr/bin/python
# Author: hiennd
# Date: 06/08/2020
# Test: PASSED
# Edit: 08/08/2020

msg = """
    Author: hiennd
    Create date: 06/08/2020
    Test: PASSED 07/08/2020
    -----------------------------------
    Node: 
        * /node_serial_mcu
    
    Published topics:
        * /robot_yaw_imu_pub    [std_msgs/Float32] 
        * /robot_vel_pub        [geometry_msgs/Twist] 

    Subscribed topics:
        * /cmd_vel              [geometry_msgs/Twist]
    
    Parameters:
        port    =   '/dev/ttyUSB_MCU'
        baud    =   256000 bps
    ------------------------------------
"""
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
import sys
#

class NodeSerialComPort:
    def __init__(self): #ok
        #Node
        rospy.init_node('node_serial_mcu')#, anonymous=True)
        #Pub
        self.rate = 50  
        self.vel_pub        = rospy.Publisher('/robot/vel_pub',      Twist,      queue_size=self.rate)
        # self.theta_pub      = rospy.Publisher('/robot/yaw_imu_pub',  Float32,    queue_size=self.rate)  
        # Note: su dung IMU MPU9250 read tu Jetson Nano
        self.voltage_pub    = rospy.Publisher('/robot/voltage_pub',  Float32,    queue_size=10)  
        
        #for logdata
        # self.odom_pub_of_mcu   = rospy.Publisher('/robot/pose_pub', Point, queue_size=self.rate) 
        #Sub
        rospy.Subscriber('/cmd_vel', Twist, self.sub_cmd_vel_callback) # def: sub_cmd_vel_callback()
        rospy.Subscriber('/robot/system_cmd', Float32, self.sub_cmd_system_callback) #/robot/system_cmd

        #Serial
        self.port_name = rospy.get_param('~port','/dev/ttyUSB_MCU')  #ttyUSB0 -> ttyUSB_MCU
        self.baud = int(rospy.get_param('~baud','256000')) #256000
        #ser = serial.Serial(port = port_name, baudrate = baud, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout=1) 
        #note: phai co timeout, neu khong function: ser.read() se blocking
        self.ser = serial.Serial(port = self.port_name, baudrate = self.baud, timeout=1)    #1s

        #data store
        self.Len_Float_rx       = 4 #int(8) # nhan 8 float = 8*4 + 1 = 33 bytes
        self.Len_Float_rx_buff  = (self.Len_Float_rx*4+1) #33 #int(self.Len_Float_rx*4 + 1) | 8*4+1 = 33 | 4*4+1 = 17
        self.data = list([])    #list empty || data.append(newdata) || data.clear() == []
        self.dataFloat = [0.0]*self.Len_Float_rx #luu data Float

        self.last_time = rospy.Time.now()
        # varible 
        self.vel_pub_msg    = Twist()       #tuong duong '/cmd_vel'
        # self.theta_pub_msg  = Float32()
        self.voltage_pub_msg = Float32()    #Save voltage of Robot
        self.nCountUpVolt = 0
        self.tx_send_ok  = bool(False)      #type bool    |0=None|1=OK=send|
        self.rx_send_ok  = bool(False)      #type bool    |0=None|1=OK=send|
        
        self.v = 0.0
        self.w = 0.0
        self.cmd = 0.0001  #01 - reset IMU | 02 = reset CPU
        self.send2mcu = False
    
    #ref: https://pyserial.readthedocs.io/en/latest/pyserial_api.html
    #ref: https://pyserial.readthedocs.io/en/latest/shortintro.html
    
    # PC send data to MCU via Serial Tx
    #PASSED
    def data_send(self):
        # 02 float need send
        #ex: Serial.write(b"\x01" + struct.pack("<ff", float1, float2))
        # == 0x01 = reset IMU (yaw)
        # == 0x02 = reset CPU.
        #TODO:
        # if int(self.cmd) == 1:
        #     #rospy.loginfo("[DEBUG][node_serial_mcu][Tx]: Send Reset IMU - Yaw to [Zero]") #PASSED-OK
        #     self.cmd = 0.0 #reset
        #     pass
        # if int(self.cmd) == 2:
        #     self.cmd = 0.0 #reset
        #     pass
            #rospy.loginfo("[DEBUG][node_serial_mcu][Tx]: Send Reset CPU [Zero]")#PASSED-OK     

        #cu goi ham nay len la cho init lai IMU
        data = pack('<fff', self.v, self.w, self.cmd) #< = litle-endian, 0.1234: du phong
        self.ser.write(data + b'\r')  #PASSED #'\r' = 0x0D = 13

        #rospy.loginfo("[DEBUG][node_serial_mcu][Tx] -- self.ser.write ---, length: = " + str(len(data + b'\r'))) #PASSED
        #self.cmd = 0.0 #reset
         
    # PC receive data from MCU via Serial Rx
    #PASSED
    def data_receive(self): #Nhan duoc data la gui luon ..... (MCU gui tam ~ 40ms / buff_data)  
        x = self.ser.read(1) #read 1byte, timeout = 1s; return: bytes
        #rospy.loginfo("[DEBUG]" + str(x))
        #rospy.loginfo("[DEBUG]" + str(xx) + str(len(xx)))
        #
        if x == '\r':   #PASSED          
            self.data.append(x)
            #rospy.loginfo("[DEBUG] 11111:" + str(len(self.data)) )
            try:
                #PASSED
                if len(self.data) == self.Len_Float_rx_buff:
                    #PASSED
                    #TODO: Khi muon thay doi so luong byte nhan: Modify : self.Len_Float_rx = dung so float cang nhan.
                    #rospy.loginfo("[DEBUG] OKOKOKKKOOK" + str(len(self.data)) )
                    for i in range(self.Len_Float_rx): # 0 -> (Len_Float_rx-1)
                        buffer = self.data[i*4:i*4+4]
                        arr= bytearray(buffer)
                        tempFloat = unpack('<f', arr)               # <type 'tuple'>(0.0,)
                        self.dataFloat[i] = list(tempFloat)[0]      # <type 'float'>0.0
                        #rospy.loginfo("[DEBUG][node_serial_mcu][Rx]: " +str(i) +": =" +str(self.dataFloat[i])) #PASSED

                    #PASSED: ~5ms (new ~ 20ms)
                    # current_time = rospy.Time.now()
                    # dt = (current_time - self.last_time).to_sec()   #deta_time
                    # self.last_time = current_time                   #update time
                    #rospy.loginfo("[DEBUG][node_serial_mcu][Rx] TimeSample: = " + str(dt) + "s, Freq = " + str(1/dt))
                    
                    #rospy.loginfo("[DEBUG][node_serial_mcu][Rx]:" + str(type(self.dataFloat[1])) + str(self.dataFloat[1]))
                    self.vel_pub_msg.linear.x    = self.dataFloat[0] #v
                    self.vel_pub_msg.angular.z   = self.dataFloat[1] #w                   
                    self.voltage_pub_msg.data    = self.dataFloat[2]#[2] = Voltage
                    # self.theta_pub_msg.data      = self.dataFloat[3]*180/math.pi # yaw_imu
                    #PASSED: 5ms - 200Hz
                    self.rx_send_ok = True  #Pub: pub_def 
                else:
                    pass
                    #Du lieu nay chinh la luc MCU khoi dong .....
                    #rospy.loginfo("[DEBUG][node_serial_mcu][Rx]:..............")     
                    #rospy.loginfo(self.data)         #: Data van bi miss nhieu....
                    self.send2mcu = False
                    for j in range(10):
                        if self.data[0] == '[':
                            self.send2mcu = True
                            break

                    #if self.data[0] == '[':
                    if self.send2mcu == True:
                        #
                        str_temp = ''
                        #for i in self.data:
                        for i in range(len(self.data)):
                            pass
                            str_temp += self.data[i]
                            #print(i),
                            #print(self.data[i])
                            #str_temp += i
                        #print(str_temp)    #PASSED
                        # rospy.loginfo("[DEBUG][node_serial_mcu][Rx]: " + str_temp) #PASSED
                        rospy.loginfo(str_temp) #PASSED

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
        # self.theta_pub.publish(self.theta_pub_msg)

    def spin(self): #ok
        rospy.loginfo("[ROS][hiennd] Start Node Serial connect to MCU at: " + self.port_name + ", " + str(self.baud) + 'bps')
        rate = rospy.Rate(100000) #100kHz == OK
        #send data phat dau de reset yaw ...
        # self.data_send()

        while not rospy.is_shutdown():
            #0. Receive(rx): cho nhan lien tuc, du data thi pub_def()       
            self.data_receive()      
            #1. Send (tx): chi khi nao, sub_cmd_vel_callback() dc goi thi send()
            if self.tx_send_ok == True:
                self.tx_send_ok = False #reset
                #
                self.data_send()
            else:
                pass
                # if self.cmd > 0:
                #     #if self.send2mcu == False:
                #     self.data_send()
                #     rospy.loginfo("[DEBUG][Tx]: Send command: " + str(self.cmd))#PASSED
                #     #else:
                #     #self.send2mcu = False
                #     self.cmd = 0.0 #reset
               
            #2. Publish to ROS Master.
            if self.rx_send_ok == True:  #ok_send
                self.rx_send_ok = False   #reset
                #
                self.pub_def()
                # khoan 1s lan pub 1 lan dien ap len
                self.nCountUpVolt += 1
                if self.nCountUpVolt >= 40: #normal: pub_def() ~ 50ms (200Hz)
                    self.nCountUpVolt = 0
                    self.voltage_pub.publish(self.voltage_pub_msg)  #PASSED ~= 1s
            #
            #rate.sleep()    # khong dung, neu dung thif rate = 100kHz
        
    
    def sub_cmd_vel_callback(self, vel_sub_msg):
        # MCU tu dong chuyen ve: wL, wR
        self.v = vel_sub_msg.linear.x        # v robot (m/s)
        self.w = vel_sub_msg.angular.z       # w robot (rad/s)
        self.tx_send_ok = True               # data ready (pre send via serial)
    
    def sub_cmd_system_callback(self, msg_data):
        self.cmd = msg_data.data
        # send data 1 phat thoi .....
        #rospy.loginfo("[ROS][hiennd] from: Node_send_system_command: cmd: = " + str(self.cmd))   #PASSED
        #self.data_send()


# -------------Main--------------------
def main():
    print(msg)
    nodeSerial = NodeSerialComPort()
    nodeSerial.spin()

if __name__ == '__main__':
    main()
