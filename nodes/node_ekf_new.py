#!/usr/bin/python

msg = """
    Author: hiennd
    ------------------------------------
"""

import rospy
import roslib
import tf

import PyKDL


import matplotlib.pyplot as plt
import numpy as np

import math
from math import sin, cos, pi
import numpy

# Messages
from std_msgs.msg import String, Int8, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3Stamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


# Covariance for EKF simulation
Q = np.diag([
    0.2,            # variance of location on x-axis
    0.2,            # variance of location on y-axis
    np.deg2rad(30)  # variance of yaw angle - 0.017453292519943295
]) ** 2             # predict state covariance

R_UWB              = np.diag([0.1, 0.1]) ** 2  # Observation x,y position covariance
#R_IMU              = 0.1
R_AMCL = np.diag([0.01, 0.01, 0.01]) ** 2 
show_animation = True

Ts = 0.1            # time tick [s] = 20ms
SIM_TIME = 50.0     # simulation time [s]

#--------------------------------------

class EKF_Publisher:
    def __init__(self):
        rospy.init_node('Node_EKF_Publisher', anonymous=True)
        #Pub
        self.odom_pub               = rospy.Publisher("odom",                   Odometry, queue_size= 50)    #50Hz
        self.odom_broadcaster_tf    = tf.TransformBroadcaster()
        self.ekf_pose_debug         = rospy.Publisher("/debug/xEst",            Point, queue_size=50)    #50Hz
        self.yaw_theta_pub          = rospy.Publisher("/debug/yaw",             Point, queue_size=50)    #50Hz
        self.xDR                    = rospy.Publisher("/debug/xDR",             Point, queue_size=50)    #50Hz
        self.amcl_init              = rospy.Publisher("/initialpose",   PoseWithCovarianceStamped, queue_size=10)
        #Sub
        rospy.Subscriber('/robot/vel_pub',      Twist,      self.vel_callback)
        rospy.Subscriber('/dwm1001c/tag',       Point,      self.uwb_sub_callback)
        rospy.Subscriber('/imu/rpy/filtered',   Vector3Stamped,    self.rpyFilter_callback)  #from MPU9250
        rospy.Subscriber('/amcl_pose',   PoseWithCovarianceStamped,    self.amcl_pose_callback)

        self.frame_id = rospy.get_param('~frame_id','/odom')    
        self.child_frame_id = rospy.get_param('~child_frame_id','/base_footprint')

        # self.pose = {'x':0.0, 'y': 0.0, 'th': 0.0}
        
        self.vel = {'v':0.0 , 'w':0.0} #v robot (m/s)  && w robot (rad/s)

        #for debug
        self.time_uwb = rospy.Time.now()
        self.time_yaw = rospy.Time.now()
        self.time_vel = rospy.Time.now()
        self.last_time  = rospy.Time.now()

        #for predict EKF
        self.time_ekf_update = rospy.Time.now()
        self.PEst = np.eye(3)*10 
        self.xEst = np.zeros((3, 1))

        # self.pose = {'x':2.19, 'y': 3.69, 'th': 0.0}
        # self.pose = {'x':1.55, 'y': 2.07, 'th': 0.0} #lab109
        self.pose = {'x':2.17, 'y': 3.7, 'th': 0.0} #hvhqy
        self.posexDR = {'x':2.17, 'y': 3.7, 'th': 0.0}
        
        #pre-updte
        self.xEst[0,0] = self.pose['x']
        self.xEst[1,0] = self.pose['y']
        self.xEst[2,0] = self.pose['th']

        self.ready = False
        self.amcl_pose_frist = False #nhan ban tin dau tien khi khoi dong
        self.nCountLidar = 0

        self.theta_amcl = 0.0
        self.yaw_imu = 0.0

        #self.item = 0

        #tinh bias khi robot dung yen
        self.bias_ready = False
        self.yaw_bias = 0.0
        self.yaw_bias_old = 0.0
        self.yaw_backup_calc_bias = 0.0
        self.yaw_bias_timevw = rospy.Time.now()

        self.drift = 0.01 #debug
        #R,Q
        self.R_IMU  = 0.1
    def update(self):
        # pass
        if self.ready == True:
            pass       
            self.pub_odometry(self.pose)
            self.pub_tf(self.pose)  
            #
            self.pub_xDR_debug(self.posexDR)
            self.pub_ekf_debug(self.pose)
            self.pub_yaw_debug()
    
    def pub_yaw_debug(self):
        yaw_theta_pub_msg = Point()
        yaw_theta_pub_msg.x = self.theta_amcl #self.posexDR['th']
        yaw_theta_pub_msg.y = self.yaw_imu 
        yaw_theta_pub_msg.z = self.pose['th']   #xEst
        self.yaw_theta_pub.publish(yaw_theta_pub_msg)
    def pub_odometry(self,pose):
        current_time = rospy.Time.now()

        # Construct odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time            # self.last_time  == current_time
        odom_msg.header.frame_id = self.frame_id        #"odom"
        odom_msg.child_frame_id = self.child_frame_id   #"base_footprint"
        # set the position
        odom_msg.pose.pose.position = Point(pose['x'], pose['y'], 0)
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))
        # set the velocity
        odom_msg.twist.twist = Twist(Vector3(self.vel['v'], 0, 0), Vector3(0, 0, self.vel['w']))
         # publish the message
        self.odom_pub.publish(odom_msg)

        #TODO: Goc theta (pose['th']) can duoc tinh toan lai (fustion voi IMU (yaw_imu))
    def pub_tf(self,pose):  #ok
        # first, we'll publish the transform over tf
        current_time = rospy.Time.now()
        self.odom_broadcaster_tf.sendTransform( \
                              (pose['x'], pose['y'], 0), \
                              tf.transformations.quaternion_from_euler(0,0,pose['th']), \
                              current_time, \
                              self.child_frame_id, \
                              self.frame_id \
                              )
    def pub_amcl_init(self,x,y,theta):
        pass
        current_time = rospy.Time.now()
        init_amcl = PoseWithCovarianceStamped()
        init_amcl.header.stamp = current_time
        init_amcl.header.frame_id = "map"
        init_amcl.pose.pose.position = Point(x, y, 0)   #ekf
        init_amcl.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,theta))
        #
        # init_amcl.pose.covariance[6*0+0] = 0.1 * 0.1
        # init_amcl.pose.covariance[6*1+1] = 0.1 * 0.1
        # init_amcl.pose.covariance[6*3+3] = math.pi/12.0 * math.pi/12.0
        #
        self.amcl_init.publish(init_amcl)


    def pub_ekf_debug(self,pose):
        pose_ = Point()
        pose_.x = pose['x']
        pose_.y = pose['y']
        pose_.z = pose['th']
        self.ekf_pose_debug.publish(pose_)
    
    def pub_xDR_debug(self,pose):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()   #deta_time
        self.last_time = current_time                   #update time

        pose['x']   += self.vel['v']*dt*cos(pose['th'] + 0.5*self.vel['w']*dt)   # += v*TIME_SAMPE*cos(theta + 0.5*w*TIME_SAMPE);
        pose['y']   += self.vel['v']*dt*sin(pose['th'] + 0.5*self.vel['w']*dt)   # += v*TIME_SAMPE*sin(theta + 0.5*w*TIME_SAMPE);
        pose['th']  += self.vel['w']*dt                                        # += w*TIME_SAMPE;
        
        pose['th'] = math.atan2(math.sin(pose['th']),math.cos(pose['th'])) # squash the orientation to between (-pi,pi)

        pose_ = Point()
        pose_.x = pose['x']
        pose_.y = pose['y']
        pose_.z = pose['th']
        self.xDR.publish(pose_)

    def vel_callback(self,vel_msg):
        pass
        self.vel['v'] = vel_msg.linear.x    # v robot (m/s)
        self.vel['w'] = vel_msg.angular.z   # w robot (rad/s)
    
    def rpyFilter_callback(self, yaw_msg):
        pass   
        #rospy.loginfo("IMU Pub -- Callback") 
        # self.drift += 0.0001#debug  
        yaw = yaw_msg.vector.z + self.drift

        tmp  = yaw - self.yaw_bias #yaw_msg.vector.z - 
        #elf.yaw_imu = tmp

        #tinh toan lai, truong hop u[v,w] == 0 thi ko fusion theo IMU qua nhieu
        if self.vel['v'] == 0 and self.vel['w'] == 0:
            pass
            #thay doi trong so cua R len 
            #self.R_IMU  = 10e8 #vo cuc
            #thoi gian nay phai keo dai vai` giay moi tinh dc bias
            current_time = rospy.Time.now()
            dt = (current_time - self.yaw_bias_timevw).to_sec() 
            #self.yaw_bias_timevw = current_time                   #update time
            if dt > 3:
                pass
                self.drift += 0.0001#debug  
                if self.bias_ready == False:
                    self.bias_ready = True
                    #backup (only once)
                    self.yaw_backup_calc_bias = yaw #yaw_msg.vector.z
                    self.yaw_bias_old = self.yaw_bias
                    
                else:
                    self.yaw_bias = yaw - self.yaw_backup_calc_bias + self.yaw_bias_old
                    print("old_bias="+str(self.yaw_bias_old)+", backup:="+str(self.yaw_backup_calc_bias) \
                    + ", current="+ str(yaw)\
                    + ", bias=" + str(self.yaw_bias) + ", temp=" + str(tmp) + ", dt=" +str(dt))

        else:
            self.bias_ready = False
            self.yaw_bias_timevw = rospy.Time.now() #reset

            self.R_IMU  = 0.1 
            u = np.array([[self.vel['v']], [self.vel['w']]])
            z = np.array([[tmp]])
            self.xEst, self.PEst = self.ekf_estimation_imu(self.xEst, self.PEst, z, u)
            
            # Update to Odometry
            self.pose['x'] = self.xEst[0,0]
            self.pose['y'] = self.xEst[1,0]
            self.pose['th'] = self.xEst[2,0]
        #
        self.ready = True  

        #debug
        #print(self.yaw_bias)     

    def uwb_sub_callback(self,uwb_msg):
        
        u = np.array([[self.vel['v']], [self.vel['w']]])
        z = np.array([[uwb_msg.x],[uwb_msg.y]])

        self.xEst, self.PEst = self.ekf_estimation_uwb(self.xEst, self.PEst, z, u) #PASSED: 100ms = 10Hz
        # Update to Odometry
        self.pose['x'] = self.xEst[0,0]
        self.pose['y'] = self.xEst[1,0]
        self.pose['th'] = self.xEst[2,0]
        #
        self.ready = True
    
    def amcl_pose_callback(self,amcl_pose_msg):
        pass
        x = amcl_pose_msg.pose.pose.position.x
        y = amcl_pose_msg.pose.pose.position.y
        rot = amcl_pose_msg.pose.pose.orientation

        self.theta_amcl = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
        #cho no khoi tao gia tri moi
        if self.amcl_pose_frist == False:
            pass          
            self.pub_amcl_init(self.pose['x'],self.pose['y'],self.pose['th'])       
            rospy.loginfo("[ROS][hiennd] AMCL_POSE Update -----x = " + str(self.pose['x']) + ", y = "+ str(self.pose['y']))
            if x > 0 and y > 0 : #truong hop x=y=nan
                self.amcl_pose_frist = True
        #Kiem tra dieu kien truoc khi fusion
        # if ((abs(x-self.pose['x']) < 0.1) and (abs(y-self.pose['y']) < 0.1) and (abs(self.theta_amcl - self.pose['th']) < np.deg2rad(5))):
        #     pass          
        #     #goi EKF o day    
        #     u = np.array([[self.vel['v']], [self.vel['w']]])
        #     z = np.array([[x],[y],[self.theta_amcl]]) #theta self.pose['th']
            
            # self.xEst, self.PEst = self.ekf_estimation_amcl(self.xEst, self.PEst, z, u)
            # # #debug: xem so lan da fusion voi Lidar
            # # self.nCountLidar += 1
            # # rospy.loginfo("[EKF][Fusion Lidar XY-Yaw]: " + str(self.nCountLidar) + ", XY-Theta:= " + str(x) + ", "+ str(y) + ", " + str(self.theta_amcl)) #PASSED
            
            # # # Update to Odometry
            # self.pose['x'] = self.xEst[0,0]
            # self.pose['y'] = self.xEst[1,0]
            # self.pose['th'] = self.xEst[2,0]
            # #
            # #if abs(self.theta_amcl - self.pose['th']) < 0.5:
            #     # self.yaw_bias = self.theta_amcl - self.yaw_imu
            # #    self.pose['th'] = self.theta_amcl
            # self.yaw_bias = abs(self.theta_amcl - self.yaw_imu)/math.pi
        #if (abs(x-self.pose['x']) > 0.1) and (abs(y-self.pose['y']) > 0.1):
        if (abs(x-self.pose['x']) > 0.05) or (abs(y-self.pose['y']) > 0.05) or (abs(self.theta_amcl - self.pose['th']) > np.deg2rad(1)):
            pass
            #init lai AMCL
            self.pub_amcl_init(self.pose['x'],self.pose['y'],self.pose['th']) 

    def spin(self): #ok
        rospy.loginfo("[ROS][hiennd] Start NodeEKF_Publisher")
        rate = rospy.Rate(50)   
        while not rospy.is_shutdown():
            self.update()            
            rate.sleep()
    #------------------------------------
    #----------------EKF (UWB) (asyn)----------------
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
    # H(UWB-IMU) [3x3] -----------------------
    def jacob_h_amcl(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]])
        return jH
    #----------------EKF-------------------------
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

        return xEst, PEst
    #-----------------
    def ekf_estimation_imu(self,xEst, PEst, z, u):
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        self.time_ekf_update = current_time                   #update time
        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + Q 

        #  Update    
        jH = self.jacob_h_imu()  #IMU ()
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + self.R_IMU
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        y = z - zPred
        xEst = xPred + K.dot(y) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

        print("xPred=" + str(xPred[2,0]) + ", imu="+ str(z) + ", y=" + str(y) + ", K=" + str(K[2,0]) + ", theta=" + str(xEst[2,0]))

        return xEst, PEst   
    #-------tao tin hieu quan sat gia cho amcl -----
    def ekf_estimation_amcl(self,xEst, PEst, z, u):
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        self.time_ekf_update = current_time                   #update time

        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + Q 

        #  Update    
        jH = self.jacob_h_amcl()  #IMU ()
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + R_AMCL
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        xEst = xPred + K.dot((z - zPred)) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

        return xEst, PEst   

# -------------Main--------------------
def main():
    print(msg)
    ekf = EKF_Publisher()
    ekf.spin()

if __name__ == '__main__':
    main()