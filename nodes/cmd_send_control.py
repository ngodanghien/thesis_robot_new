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

rospy.init_node('node_send_system_command', anonymous=True)
rospy.loginfo("[ROS][hiennd] Start node_send_system_command: Disable Control PID")
command_pub   = rospy.Publisher('/robot/system_cmd',  Float32,    queue_size=50) 

mode_run = rospy.get_param('~mode_control','enable')    #disable|enable|imu|mcu

cmd_value = 0.0
if mode_run == 'imu':
    cmd_value = 1.00001 # reset IMU
if mode_run == 'mcu':
    cmd_value = 2.00001 # reset MCU
if mode_run == 'disable':
    cmd_value = 3.00001 # disable # Disable Control PID./
if mode_run == 'enable':
    cmd_value = 4.00001 #resume

command_pub_msg  = Float32()
command_pub_msg.data = cmd_value 

#command_pub.publish(command_pub_msg) #once


nCount = 0
rate = rospy.Rate(10) #100kHz == OK
while not rospy.is_shutdown() and nCount < 5:
    nCount += 1
    command_pub.publish(command_pub_msg)
    rospy.loginfo("[ROS][hiennd] Start node_send_system_command: " + mode_run + ", value = " + str(cmd_value))
    rate.sleep()
