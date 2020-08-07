#!/usr/bin/python
# Author: hiennd
# Date: 07/08/2020
# Test: ???

msg = """
    Author: hiennd
    Create date: 06/08/2020
    Test: PASSED ???
    -----------------------------------
    Node: 
        * /odometry_publisher
    Published topics:
    Subscribed topics:
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

class Nav_Circle:
    def __init__(self):
        pass
        #Node
        rospy.init_node('node_nav_circel', anonymous=True)
        #Pub
        self.rate = 50     #Hz
        #self.vel_pub     = rospy.Publisher('robot_vel_pub',      Twist,      queue_size=self.rate)
        #self.theta_pub   = rospy.Publisher('robot_yaw_imu_pub',  Float32,    queue_size=self.rate)   
        #Sub
        #rospy.Subscriber('/cmd_vel', Twist, self.sub_callback) 
            
    def spin(self): #ok
        rospy.loginfo("[ROS][hiennd] Start Navigation Circle")
        rate = rospy.Rate(self.rate) 
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()   
    
    def sub_cmd_vel_callback(self, sub_msg):
        pass
    
    def update(self):
        pass
# -------------Main--------------------
def main():
    print(msg)
    nav = Nav_Circle()
    nav.spin()

if __name__ == '__main__':
    main()
    