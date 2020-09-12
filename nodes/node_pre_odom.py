#!/usr/bin/python
# Author: hiennd
# Date: 06/08/2020
# Test: PASSED (06/08/2020)

msg = """
    Author: hiennd
    Create date: 06/08/2020
    Test: PASSED 07/08/2020
    -----------------------------------
    Node: 
        * /odometry_publisher
    
    Published topics:
        * /odom             [nav_msgs/Odometry]
        * /tf               [tf2_msgs/TFMessage]

    Subscribed topics:
        * /robot_vel_pub    [geometry_msgs/Twist]
    
    Parameters:
        * frame_id          =   '/odom'
        * child_frame_id    =   '/base_footprint'
    ------------------------------------
"""


import rospy
import roslib
import tf

import math
from math import sin, cos, pi
import numpy


# Messages
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class OdomPublisher:
    def __init__(self): #ok
        #node
        rospy.init_node('odometry_publisher')
        #pub
        self.rate = 50      #50 Hz

        # self.odom_pub               = rospy.Publisher("odom", Odometry, queue_size=self.rate)    #50Hz
        # self.odom_broadcaster_tf    = tf.TransformBroadcaster()

        # self.wheel_left_broadcaster_tf    = tf.TransformBroadcaster() # tf <-> : base_link -> wheel_left_link
        # self.wheel_right_broadcaster_tf    = tf.TransformBroadcaster() # tf <-> : base_link -> wheel_right_link

        #sub from note (receive data from MCU send up) : node_serial_rx.
        rospy.Subscriber('/robot/vel_pub',     Twist,      self.vel_callback)
        rospy.Subscriber('/robot/yaw_imu_pub',   Float32,    self.yaw_callback)
        #param : Note: Neu server ko co thi` ros tu gan: frame_id = '/odom'
        # self.L = rospy.get_param('~robot_wheel_separation_distance', 0.37)
        # self.R = rospy.get_param('~robot_wheel_radius', 0.0625)
        # self.rate = rospy.get_param('~rate', 50)

        # self.frame_id = rospy.get_param('~frame_id','/odom')    
        # self.child_frame_id = rospy.get_param('~child_frame_id','/base_footprint')

        #varible
        # self.pose = {'x':0, 'y': 0, 'th': 0}

        self.v = 0  #vx = v; vy = 0;
        self.w = 0
        
        self.last_time = rospy.Time.now()


    def spin(self): #ok
        rospy.loginfo("[ROS][hiennd] Start odometry_publisher (topic: /odom & /tf)")
        rate = rospy.Rate(self.rate)  #chuan 50Hz (tested) cho ca /odom vs /tf
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def pub_odometry(self,pose):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()   #deta_time
        self.last_time = current_time                   #update time
        
        # compute odometry in a typical way given the velocities of the robot

        #new code: hiennd == code in MCU == code in matlab

        pose['x']   += self.v*dt*cos(pose['th'] + 0.5*self.w*dt)   # += v*TIME_SAMPE*cos(theta + 0.5*w*TIME_SAMPE);
        pose['y']   += self.v*dt*sin(pose['th'] + 0.5*self.w*dt)   # += v*TIME_SAMPE*sin(theta + 0.5*w*TIME_SAMPE);
        pose['th']  += self.w*dt                                        # += w*TIME_SAMPE;
        
        pose['th'] = math.atan2(math.sin(pose['th']),math.cos(pose['th'])) # squash the orientation to between (-pi,pi)

        # Construct odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time            # self.last_time  == current_time
        odom_msg.header.frame_id = self.frame_id        #"odom"
        odom_msg.child_frame_id = self.child_frame_id   #"base_footprint"
        # set the position
        odom_msg.pose.pose.position = Point(pose['x'], pose['y'], 0)
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))
        # set the velocity
        odom_msg.twist.twist = Twist(Vector3(self.v, 0, 0), Vector3(0, 0, self.w))
         # publish the message
        self.odom_pub.publish(odom_msg)

        #TODO: Goc theta (pose['th']) can duoc tinh toan lai (fustion voi IMU (yaw_imu))
    def pub_tf(self,pose):  #ok
        # first, we'll publish the transform over tf
        self.odom_broadcaster_tf.sendTransform( \
                              (pose['x'], pose['y'], 0), \
                              tf.transformations.quaternion_from_euler(0,0,pose['th']), \
                              self.last_time, \
                              self.child_frame_id, \
                              self.frame_id \
                              )
        #base_link -> wheel_left_link
        # self.wheel_left_broadcaster_tf.sendTransform( \
        #                       (pose['x'], pose['y'], 0), \
        #                       tf.transformations.quaternion_from_euler(0,0,pose['th']), \
        #                       self.last_time, \
        #                       "", \
        #                       "wheel_left_link"\
        #                       )
        # #base_link -> wheel_right_link
        # self.wheel_right_broadcaster_tf.sendTransform( \
        #                       (pose['x'], pose['y'], 0), \
        #                       tf.transformations.quaternion_from_euler(0,0,pose['th']), \
        #                       self.last_time, \
        #                       "", \
        #                       "wheel_right_link" \
        #                       )
    
    #Update
    def update(self):     #ok
        # self.pub_odometry(self.pose)
        # self.pub_tf(self.pose)    
    
    # function Callback: from calc odometry
    def vel_callback(self,vel): #ok
        self.v = vel.linear.x    # v robot (m/s) 
        self.w = vel.angular.z   # w robot (rad/s)

    # function Callback: yaw from MPU6050
    def yaw_callback(self,yaw): #ok
        self.yaw_imu = yaw.data

def main():
    print(msg)
    odom_publisher = OdomPublisher()
    odom_publisher.spin()

if __name__ == '__main__':
    main()
