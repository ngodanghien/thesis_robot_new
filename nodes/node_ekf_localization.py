#!/usr/bin/python
# Author: hiennd
# Date: 08/08/2020
# Test: ???

msg = """
    Author: hiennd
    ------------------------------------
"""


import rospy
import roslib
import tf

import math
import matplotlib.pyplot as plt
import numpy as np

import math
from math import sin, cos, pi
import numpy

# Messages
from std_msgs.msg import String, Int8, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Covariance for EKF simulation
Q = np.diag([
    0.1,            # variance of location on x-axis
    0.1,            # variance of location on y-axis
    np.deg2rad(30)  # variance of yaw angle - 0.017453292519943295
]) ** 2             # predict state covariance

R_UWB              = np.diag([0.03, 0.024]) ** 2  # Observation x,y position covariance
R_IMU           = 0.01
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
        self.ekf_pose_debug                 = rospy.Publisher("/debug/ekf_pose_debug", Point, queue_size=self.rate)    #50Hz
        self.yaw_imu_drift_debug            = rospy.Publisher("/debug/yaw_imu_sub_drift", Float32, queue_size=self.rate)
        #self.deadReckoning_pose_debug         = rospy.Publisher("dead_reckoning_pose_debug", Point, queue_size=self.rate)
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

        #for EKF
        self.xTrue = np.zeros((3, 1))  # <=>  self.pose = {'x':0.0, 'y': 0.0, 'th': 0.0}
        self.PEst = np.eye(3)*10 
        self.xEst = np.zeros((3, 1))
        self.xDR = np.zeros((3, 1))      # Dead reckoning
        self.time_ekf_update        = rospy.Time.now()
        self.time_ekf_update_imu    = rospy.Time.now()
        #save goc yaw ban dau, truoc khi EKF chay, ly do yaw bi troi (!= 0)

        self.drift_yaw      = 0
        self.nCount         = 0

    def spin(self): #ok
        rospy.loginfo("[ROS][hiennd] Start NodeEKF_Publisher")
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()            
            rate.sleep()

    def update(self):
        pass
        #self.dead_reckoning() #debug
        #final (after EKF Fusion)
        self.pub_odometry(self.pose)
        self.pub_tf(self.pose) 

    #don gian chi can cho pose = xEst cua Kalman la dc.
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
        # pose['x'] = self.pose_deadReckoning['x']
        # pose['y'] = self.pose_deadReckoning['y']
        # pose['th'] = self.pose_deadReckoning['th']
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
        
        
        self.yaw_imu['th'] = imu_msg.data - self.drift_yaw   # Float32
        # rospy.loginfo("[DEBUG][imu_sub_callback]---------------: " + str(self.yaw_imu['th']))                                     #PASSED
        data = Float32()
        data = self.yaw_imu['th']
        self.yaw_imu_drift_debug.publish(data) #debug
        
        u = np.array([[self.vel['v']], [self.vel['w']]])
        z = np.array([[self.yaw_imu['th']]])

        #TODO:

        #cho nhan vai ban tin tu IMU: self.drift_yaw
        # de update gia tri yaw dang troi .....
        self.nCount += 1
        if self.nCount > 5:
            self.xEst, self.PEst = self.ekf_estimation_imu(self.xEst, self.PEst, z, u)
        else:
            pass
            rospy.loginfo("[DEBUG][imu_sub_callback]----(Dang Calib goc troi Yaw)-----: " + str(z))
            self.drift_yaw = imu_msg.data

    def vel_sub_callback(self, vel_msg):
        pass
        self.vel['v'] = vel_msg.linear.x    # v robot (m/s)
        self.vel['w'] = vel_msg.angular.z   # w robot (rad/s)
        # rospy.loginfo("[DEBUG][vel_sub_callback]---------------: " + str(self.vel['v']) + ", " + str(self.vel['w']))               #PASSED
    
    def uwb_sub_callback(self, uwb_msg):
        pass
        self.xy_uwb_tag['x'] = uwb_msg.x
        self.xy_uwb_tag['y'] = uwb_msg.y
        #rospy.loginfo("[DEBUG][uwb_sub_callback]---------------: " + str(self.xy_uwb_tag['x']) + ", " + str(self.xy_uwb_tag['y'])) #PASSED
        #cu co data la update
        u = np.array([[self.vel['v']], [self.vel['w']]])
        z = np.array([[uwb_msg.x],[uwb_msg.y]])

        #TODO:
        self.xEst, self.PEst = self.ekf_estimation_uwb(self.xEst, self.PEst, z, u) #PASSED: 100ms = 10Hz

    # def ekf_predict(self,pose): #DeadReckoning
    #     current_time = rospy.Time.now()
    #     dt = (current_time - self.last_time).to_sec()   #deta_time
    #     self.last_time = current_time                   #update time
        
    #     # compute odometry in a typical way given the velocities of the robot

    #     #new code: hiennd == code in MCU == code in matlab
    #     x       = self.vel['v']*dt*cos(pose['th'] + 0.5*self.vel['w']*dt)
    #     y       = self.vel['v']*dt*sin(pose['th'] + 0.5*self.vel['w']*dt)
    #     theta   = self.vel['w']*dtekf_predict
    #     # pose['x']   += self.vel['v']*dt*cos(pose['th'] + 0.5*self.vel['w']*dt)   # += v*TIME_SAMPE*cos(theta + 0.5*w*TIME_SAMPE);
    #     # pose['y']   += self.vel['v']*dt*sin(pose['th'] + 0.5*self.vel['w']*dt)   # += v*TIME_SAMPE*sin(theta + 0.5*w*TIME_SAMPE);
    #     # pose['th']  += self.vel['w']*dt                                        # += w*TIME_SAMPE;
        
    #     pose['th'] = math.atan2(math.sin(pose['th']),math.cos(pose['th'])) # squash the orientation to between (-pi,pi)
    
    # robot system model --------------
    def robot_model_system(self,x, u,Ts):
        # Ref: (2.20) chapter (2.2.2)
        w = u[1,0]
        theta = x[2,0]
        # F = [3x3]
        F = np.array([[1.0, 0, 0],
                    [0, 1.0, 0],
                    [0, 0, 1.0]])  
        # B = [3x2]
        B = np.array([[Ts*math.cos(theta + 0.5*Ts*w),       0],
                    [Ts*math.sin(theta + 0.5*Ts*w),         0],
                    [0.0,                                   Ts]])
        # = [3x3][3x1] + [3x2][2x1] = [3x1]
        #x = F @ x + B @ u
        x = F.dot(x) + B.dot(u)

        #
        return x
    # Matrix Fk [3x3] -------------------------------
    def jacob_f(self,x, u,Ts):
        v       = u[0, 0]     # v - robot
        w       = u[1, 0]     # w - robot
        theta   = x[2,0]      # yaw
        #jF = 3x3
        jF = np.array([
            [1.0,   0.0,    -Ts * v * math.sin(theta + 0.5*Ts*w)],
            [0.0,   1.0,    -Ts * v * math.cos(theta + 0.5*Ts*w)],
            [0.0,   0.0,                                    1.0]])
        return jF

    def ekf_update_imu():
        pass
        self.ekf_predict(self.pose)
    
    def ekf_update_uwb():
        pass
        #xTrue, z, xDR, ud = observation_uwb(xTrue, xDR, u)  #uwb 
        #Nhan du lieu tu UWB ...

        #xEst, PEst = ekf_estimation_uwb(xEst, PEst, z, ud)

    #----------------EKF (UWB) (asyn)----------------
    def ekf_estimation_uwb(self,xEst, PEst, z, u):
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        self.time_ekf_update = current_time                   #update time
        #rospy.loginfo("[DEBUG][ekf_estimation_uwb]---------------: " + str(Ts)) #PASSED
        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + Q 

        #  Update
        jH = self.jacob_h_uwb()
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + R_UWB
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        xEst = xPred + K.dot((z - zPred)) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)


        #rospy.loginfo("[DEBUG][ekf_estimation_uwb]---------------: " + str(xEst)) 

        # Update to Odometry
        self.pose['x'] = xEst[0,0]
        self.pose['y'] = xEst[1,0]
        self.pose['th'] = xEst[2,0]

        return xEst, PEst
    def ekf_estimation_imu(self,xEst, PEst, z, u):
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update_imu).to_sec()   #deta_time
        self.time_ekf_update_imu = current_time                   #update time
        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + Q 

        #  Update    
        jH = self.jacob_h_imu()  #IMU ()
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + R_IMU
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        xEst = xPred + K.dot((z - zPred)) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)


        #rospy.loginfo("[DEBUG][ekf_estimation_imu]---------------: " + str(xEst)) 

        return xEst, PEst

    def jacob_h_uwb(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0],
            [0, 1, 0] ])
        return jH
    def jacob_h_imu(self):
        # Jacobian of Observation Model
        jH = np.array([
            [0, 0, 1]])   
        return jH

    def dead_reckoning(self):
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
        #self.deadReckoning_pose_debug.publish(deadReckoning_msg)

# -------------Main--------------------
def main():
    print(msg)
    ekf = EKF_Publisher()
    ekf.spin()

if __name__ == '__main__':
    main()
