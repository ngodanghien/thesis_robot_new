#!/usr/bin/python
# Author: hiennd
# Date: 15/08/2020
# Test: (ko reset MCU duoc khi dang cam vao Jetson va cap nguon them ngoai)
#        Neu only sd 01 nguon tu Mach nap thi reset binh thuong.
msg = """
    Author: hiennd
    Create date: 15/08/2020
    Test: PASSED OK
    -----------------------------------
    Node: 
        * /node_send_system_command
    Published topics:
"""

# ROS
import rospy
import roslib
# Messages
from std_msgs.msg import Float32

rospy.init_node('node_send_system_command_mcu', anonymous=True)
rospy.loginfo("[ROS][hiennd] Start node_send_system_command: RESET IMU or MCU")
command_pub   = rospy.Publisher('/robot/system_cmd',  Float32,    queue_size=50) 

command_pub_msg  = Float32()
command_pub_msg.data = 2.00001 #reset CPU


nCount = 0
rate = rospy.Rate(10) #100kHz == OK
while not rospy.is_shutdown():
    nCount += 1
    command_pub.publish(command_pub_msg)
    rospy.loginfo("[ROS][hiennd] Start node_send_system_command: RESET IMU or MCU")
    rate.sleep()

    if nCount >= 10:
        pass 
        break