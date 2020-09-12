#!/usr/bin/python
# Author: hiennd
# Date: 15/08/2020
# Test: ???

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

rospy.init_node('node_send_system_command_imu', anonymous=True)
rospy.loginfo("[ROS][hiennd] Start node_send_system_command: RESET IMU or MCU")
command_pub   = rospy.Publisher('/robot/system_cmd',  Float32,    queue_size=50) 

command_pub_msg  = Float32()
command_pub_msg.data = 1.00001 #reset IMU

command_pub.publish(command_pub_msg) #once


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