#!/usr/bin/python
# Author: hiennd
# Date: 08/08/2020
# Test: ???

msg = """
    Author: hiennd
    Create date: 08/08/2020
    Test: PASSED 08/08/2020
    -----------------------------------
    Node:
        * /Node_EKF_Publisher
    Published topics:
        * /odom                 [nav_msgs/Odometry]
        * /tf                   [tf2_msgs/TFMessage]
    Subscribed topics:
        * /robot_yaw_imu_pub    [std_msgs/Float32]
        * /robot/vel_pub        [geometry_msgs/Twist]
        * /dwm1001c/tag         [localizer_dwm1001/Tag]
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
from std_msgs.msg import String, Int8, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class EKF_Publisher:
    def __init__(self):
        pass
        #Node
        rospy.init_node('Node_EKF_Publisher', anonymous=True)
        #Pub
        self.rate = 50      #50 Hz
        # Thay the cho node_odom.py --->
        self.frame_id       = rospy.get_param('~frame_id',      '/odom')    
        self.child_frame_id = rospy.get_param('~child_frame_id','/base_footprint')

        self.odom_pub               = rospy.Publisher("odom", Odometry, queue_size=self.rate)    #50Hz
        self.odom_broadcaster_tf    = tf.TransformBroadcaster()
        #Debug
        self.ekf_pose_debug         = rospy.Publisher("ekf_pose_debug", Point, queue_size=self.rate)    #50Hz
        self.deadReckoning_pose_debug         = rospy.Publisher("dead_reckoning_pose_debug", Point, queue_size=self.rate)
        #Sub
        rospy.Subscriber('/robot/yaw_imu_pub',  Float32,    self.imu_sub_callback)
        rospy.Subscriber('/robot/vel_pub',      Twist,      self.vel_sub_callback)
        rospy.Subscriber('/dwm1001c/tag',       Point,      self.uwb_sub_callback)
        #Variable
        self.pose = {'x':0.0, 'y': 0.0, 'th': 0.0}
        self.pose_deadReckoning = {'x':0.0, 'y': 0.0, 'th': 0.0}
        self.vel = {'v':0.0 , 'w':0.0} #v robot (m/s)  && w robot (rad/s)

        self.yaw_imu = {'th':0.0}
        self.xy_uwb_tag = {'x':0.0, 'y':0.0}
        #
        self.last_time = rospy.Time.now()
        self.last_time_dead_reckoning = rospy.Time.now() #deadReckoning_pose_debug

    def spin(self): #ok
        rospy.loginfo("[ROS][hiennd] Start NodeEKF_Publisher")
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()            
            rate.sleep()

    def update(self):
        pass
        self.dead_reckoning() #debug
        #final (after EKF Fusion)
        self.pub_odometry(self.pose)
        self.pub_tf(self.pose) 


    def pub_odometry(self,pose):
        pass
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()   #deta_time
        self.last_time = current_time                   #update time
        # Construct odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.last_time          # self.last_time  == current_time
        odom_msg.header.frame_id = self.frame_id        #"odom"
        odom_msg.child_frame_id = self.child_frame_id   #"base_footprint"
        
        #dubug - demo
        pose['x'] = self.pose_deadReckoning['x']
        pose['y'] = self.pose_deadReckoning['y']
        pose['th'] = self.pose_deadReckoning['th']
        #end- debug
        
        # set the position
        odom_msg.pose.pose.position = Point(pose['x'], pose['y'], 0)
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))
        # set the velocity
        odom_msg.twist.twist = Twist(Vector3(self.vel['v'], 0, 0), Vector3(0, 0, self.vel['w']))
         # publish the message
        self.odom_pub.publish(odom_msg)

        #debug  == estimation EKF
        pose_ = Point()
        pose_.x = pose['x']
        pose_.y = pose['y']
        pose_.z = pose['th']
        self.ekf_pose_debug.publish(pose_)
        #end debug
    def pub_tf(self,pose):  #ok
        # first, we'll publish the transform over tf
        self.odom_broadcaster_tf.sendTransform( \
                              (pose['x'], pose['y'], 0), \
                              tf.transformations.quaternion_from_euler(0,0,pose['th']), \
                              self.last_time, \
                              self.child_frame_id, \
                              self.frame_id \
                              )
    
    def imu_sub_callback(self, imu_msg):
        pass
        self.yaw_imu['th'] = imu_msg.data   # Float32
        # rospy.loginfo("[DEBUG][imu_sub_callback]---------------: " + str(self.yaw_imu['th']))                                     #PASSED

    def vel_sub_callback(self, vel_msg):
        pass
        self.vel['v'] = vel_msg.linear.x    # v robot (m/s)
        self.vel['w'] = vel_msg.angular.z   # w robot (rad/s)
        # rospy.loginfo("[DEBUG][vel_sub_callback]---------------: " + str(self.vel['v']) + ", " + str(self.vel['w']))               #PASSED
    
    def uwb_sub_callback(self, uwb_msg):
        pass
        self.xy_uwb_tag['x'] = uwb_msg.x
        self.xy_uwb_tag['y'] = uwb_msg.y
        # rospy.loginfo("[DEBUG][uwb_sub_callback]---------------: " + str(self.xy_uwb_tag['x']) + ", " + str(self.xy_uwb_tag['y'])) #PASSED
    
    def ekf_predict(self,pose): #DeadReckoning
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()   #deta_time
        self.last_time = current_time                   #update time
        
        # compute odometry in a typical way given the velocities of the robot

        #new code: hiennd == code in MCU == code in matlab
        x       = self.vel['v']*dt*cos(pose['th'] + 0.5*self.vel['w']*dt)
        y       = self.vel['v']*dt*sin(pose['th'] + 0.5*self.vel['w']*dt)
        theta   = self.vel['w']*dtekf_predict
        # pose['x']   += self.vel['v']*dt*cos(pose['th'] + 0.5*self.vel['w']*dt)   # += v*TIME_SAMPE*cos(theta + 0.5*w*TIME_SAMPE);
        # pose['y']   += self.vel['v']*dt*sin(pose['th'] + 0.5*self.vel['w']*dt)   # += v*TIME_SAMPE*sin(theta + 0.5*w*TIME_SAMPE);
        # pose['th']  += self.vel['w']*dt                                        # += w*TIME_SAMPE;
        
        pose['th'] = math.atan2(math.sin(pose['th']),math.cos(pose['th'])) # squash the orientation to between (-pi,pi)

    
    def ekf_update_imu():
        pass
        self.ekf_predict(self.pose)
    
    def ekf_update_uwb():
        pass

    def dead_reckoning(self):
        #TODO
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time_dead_reckoning).to_sec()   #deta_time
        self.last_time_dead_reckoning = current_time                   #update time
        
        # compute odometry in a typical way given the velocities of the robot

        #new code: hiennd == code in MCU == code in matlab
        theta   = self.pose_deadReckoning['th']
        x       = self.vel['v']*dt*cos(theta + 0.5*self.vel['w']*dt)
        y       = self.vel['v']*dt*sin(theta + 0.5*self.vel['w']*dt)
        theta   = self.vel['w']*dt

        # deadReckoning_pose_debug
        self.pose_deadReckoning['x']    += x
        self.pose_deadReckoning['y']    += y
        self.pose_deadReckoning['th']   += theta
        self.pose_deadReckoning['th']   = math.atan2(sin(self.pose_deadReckoning['th']),cos(self.pose_deadReckoning['th'])) # squash the orientation to between (-pi,pi)
        
        deadReckoning_msg = Point()
        deadReckoning_msg.x = self.pose_deadReckoning['x']
        deadReckoning_msg.y = self.pose_deadReckoning['y']
        deadReckoning_msg.z = self.pose_deadReckoning['th']
        #debug
        self.deadReckoning_pose_debug.publish(deadReckoning_msg)
        
# -------------Main--------------------
def main():
    print(msg)
    ekf = EKF_Publisher()
    ekf.spin()

if __name__ == '__main__':
    main()
