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
# Q = np.diag([
#     0.2,            # variance of location on x-axis
#     0.2,            # variance of location on y-axis
#     np.deg2rad(30)  # variance of yaw angle - 0.017453292519943295
# ]) ** 2             # predict state covariance
#Q1 = np.diag([0.4, 0.5])** 2    #v,w
# R_UWB              = np.diag([0.31, 0.31]) ** 2  # Observation x,y position covariance
#R_IMU              = 0.1


#--------------------------------------

class EKF_Publisher:
    def __init__(self):
        rospy.init_node('Node_EKF_Publisher_Debug', anonymous=True)
        #Pub
        #Pub
        self.odom_pub               = rospy.Publisher("odom",                   Odometry, queue_size= 50)    #50Hz
        self.odom_broadcaster_tf    = tf.TransformBroadcaster()
        self.ekf_pose_debug         = rospy.Publisher("/debug/xEst_new",            Point, queue_size=50)    #50Hz
        self.xDR                    = rospy.Publisher("/debug/xDR_new",            Point, queue_size=50)    #50Hz     
        self.yaw_theta_pub          = rospy.Publisher("/debug/yaw_new",             Point, queue_size=50)    #50Hz
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
        self.PEst_UWB   = np.eye(3)*10 
        self.PEst_IMU   = np.eye(3)*10 
        self.PEst_AMCL  = np.eye(3)*10 
        self.xEst = np.zeros((3, 1))

        # self.pose = {'x':2.19, 'y': 3.69, 'th': 0.0}
        # self.pose = {'x':1.55, 'y': 2.07, 'th': 0.0} #lab109
        #self.pose = {'x':2.17, 'y': 3.7, 'th': 0.0} #hvhqy
        #self.posexDR = {'x':2.17, 'y': 3.7, 'th': 0.0}

        #test lai vong tron : 2020-08-19-04-28-55.bag || 2020-08-19-04-41-31.bag
        # self.pose = {'x':2.15, 'y': 5.0, 'th': 0.0}
        # self.posexDR = {'x':2.15, 'y': 5.0, 'th': 0.0}

        #test lai hinh vuong : 2020-08-19-04-57-13.bag || 2020-08-19-05-27-36.bag
        # self.pose = {'x':1.56, 'y': 3.14, 'th': 0.0}
        # self.posexDR = {'x':1.56, 'y': 3.14, 'th': 0.0}
        
        #Chu P: 2020-08-19-05-27-36.bag
        # self.pose = {'x':0.97, 'y': 1.30, 'th': 0.0}
        # self.posexDR = {'x':0.97, 'y': 1.30, 'th': 0.0}

        #Full sensor - vong tron nho | to : 2020-08-27-04-59-08.bag || 2020-08-27-04-50-29.bag
        # self.pose = {'x':2.17, 'y': 3.70, 'th': 0.0}
        # self.posexDR = {'x':2.17, 'y': 3.70, 'th': 0.0}
        self.pose = {'x':0.46, 'y': 0.95, 'th': 0.0}
        self.posexDR = {'x':0.46, 'y': 0.95, 'th': 0.0}

        #Dieu huong: di cheo
        #2020-08-28-00-29-57.bag
        # self.pose = {'x':2.17, 'y': 3.7, 'th': 0.0} #hvhqy
        # self.posexDR = {'x':2.17, 'y': 3.7, 'th': 0.0}

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
        self.R_IMU          = 0.1
        self.R_UWB          = np.diag([0.31, 0.31]) ** 2
        self.R_UWB_IMU      = np.diag([0.31, 0.31, 0.01]) ** 2
        self.R_AMCL         = np.diag([0.01, 0.01, 0.01]) ** 2 

        self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2 # OK predict state covariance
        self.Q1 = np.diag([0.4, 0.5])** 2    #v,w

        self.checkUWB = False   #kiem tra lan nhau dau tien
    def update(self):
        # pass
        if self.ready == True:
            pass       
            self.pub_odometry(self.pose)
            self.pub_tf(self.pose)
            self.pub_ekf_debug(self.pose)
            self.pub_xDR_debug(self.posexDR)
            self.pub_yaw_debug()
    
    def pub_yaw_debug(self):
        yaw_theta_pub_msg = Point()
        yaw_theta_pub_msg.x = self.posexDR['th'] #self.theta_amcl 
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
        self.yaw_imu = yaw_msg.vector.z
        pass
        # current_time = rospy.Time.now()
        # Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        # self.time_ekf_update = current_time                   #update time 
        # #

        # #tinh toan lai, truong hop u[v,w] == 0 thi ko fusion theo IMU qua nhieu
        # if self.vel['v'] == 0 and self.vel['w'] == 0:
        #     # pass
        #     # # thoi gian nay phai keo dai vai` giay moi tinh dc bias
        #     # # current_time = rospy.Time.now()
        #     # # dt = (current_time - self.yaw_bias_timevw).to_sec() 
        #     # # #self.yaw_bias_timevw = current_time                   #update time
        #     # # if dt > 3:
        #     # #     pass
        #     # #     #self.drift += 0.0001#debug  
        #     # #     if self.bias_ready == False:
        #     # #         self.bias_ready = True
        #     # #         #backup (only once)
        #     # #         self.yaw_backup_calc_bias = yaw #yaw_msg.vector.z
        #     # #         self.yaw_bias_old = self.yaw_bias
                    
        #     # #     else:
        #     # #         self.yaw_bias = yaw - self.yaw_backup_calc_bias + self.yaw_bias_old
        #     # #         # print("old_bias="+str(self.yaw_bias_old)+", backup:="+str(self.yaw_backup_calc_bias) \
        #     # #         # + ", current="+ str(yaw)\
        #     # #         # + ", bias=" + str(self.yaw_bias) + ", temp=" + str(tmp) + ", dt=" +str(dt))
        #     self.R_IMU  = 10e8
        #     self.Q = np.diag([  0.001, 0.001, 0.001]) ** 2 # predict state covariance
        # else:
        #     pass
        #     self.R_IMU  = 0.01
        #     #self.Q = np.diag([  0.0609, 0.0609, 0.0275]) ** 2 # predict state covariance
        #     self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2 # predict state covariance
            
        # u = np.array([[self.vel['v']], [self.vel['w']]])
        # z = np.array([[self.yaw_imu]])
        # self.xEst, self.PEst_IMU  = self.ekf_estimation_imu(self.xEst, self.PEst_IMU, z, u, Ts)
            
        # # Update to Odometry
        # self.pose['x']  = self.xEst[0,0]
        # self.pose['y']  = self.xEst[1,0]
        # self.pose['th'] = self.xEst[2,0]
        # #
        # self.ready = True     

    def uwb_sub_callback(self,uwb_msg):
        pass
        #update time
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        self.time_ekf_update = current_time                   #update time
        #
        u = np.array([[self.vel['v']], [self.vel['w']]])
        self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2 # Day chac la tham so tot nhat
        self.R_UWB_IMU      = np.diag([0.31, 0.31, 0.001]) ** 2
        if self.vel['v'] == 0 and self.vel['w'] == 0:
            pass
            self.Q = np.diag([  0.01, 0.01, np.deg2rad(0.01)]) ** 2 # OK predict state covariance
            self.R_UWB_IMU      = np.diag([0.31, 0.31, 10e4]) ** 2
        else:
            pass
            self.R_UWB_IMU      = np.diag([0.31, 0.31, 0.001]) ** 2
        
        if self.checkUWB == True: #bo qua lan dau tien ko EKF Fusion
            # self.Q = np.diag([  0.31, 0.31, np.deg2rad(10)]) ** 2 # 0.17453292519943295**2
            # z = np.array([[uwb_msg.x],[uwb_msg.y]])
            # self.xEst, self.PEst_UWB = self.ekf_estimation_uwb(self.xEst, self.PEst_UWB, z, u, Ts) #PASSED: 100ms = 10Hz
            z = np.array([[uwb_msg.x],[uwb_msg.y],[self.yaw_imu]])
            self.xEst, self.PEst_UWB = self.ekf_estimation_uwb_imu(self.xEst, self.PEst_UWB, z, u, Ts) #PASSED: 100ms = 10Hz
                
            # Update to Odometry
            self.pose['x'] = self.xEst[0,0]
            self.pose['y'] = self.xEst[1,0]
            self.pose['th'] = self.xEst[2,0]
            #
        self.ready = True
        self.checkUWB = True
    
    def amcl_pose_callback(self,amcl_pose_msg):
        pass
        # current_time = rospy.Time.now()
        # Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        # self.time_ekf_update = current_time                   #update time
        # pass
        x = amcl_pose_msg.pose.pose.position.x
        y = amcl_pose_msg.pose.pose.position.y
        rot = amcl_pose_msg.pose.pose.orientation
        theta= PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
        
        #cho no khoi tao gia tri moi (phan nay da co san roi ...)
        if self.amcl_pose_frist == False:
            pass          
            # self.pub_amcl_init(self.pose['x'],self.pose['y'],self.pose['th'])       
            # rospy.loginfo("[ROS][hiennd] AMCL_POSE Update -----x = " + str(self.pose['x']) + ", y = "+ str(self.pose['y']))
            # if x > 0 and y > 0 : #truong hop x=y=nan
            #     self.amcl_pose_frist = True
        if (abs(x-self.pose['x']) > 0.01) or (abs(y-self.pose['y']) > 0.01) or (abs(self.theta_amcl - self.pose['th']) > np.deg2rad(0.1)):
            pass
            #init lai AMCL
            self.pub_amcl_init(self.pose['x'],self.pose['y'],self.pose['th']) 
        
        # # #    

        # # print("amcl_pose_callback: x = "+str(x) +", y="+str(y)+", theta="+str(theta))

        # #if (abs(x-self.pose['x']) < 0.05) and (abs(y-self.pose['y']) < 0.05) and (abs(self.theta_amcl - self.pose['th']) < np.deg2rad(5)):

        # self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2 # Day chac la tham so tot nhat
        # self.R_UWB_IMU      = np.diag([0.21, 0.21, 0.002]) ** 2
            
        # u = np.array([[self.vel['v']], [self.vel['w']]])
        # z = np.array([[x],[y],[theta]]) 

        # self.xEst, self.PEst_AMCL = self.ekf_estimation_amcl(self.xEst, self.PEst_AMCL, z, u, Ts)     
            
        # # Update to Odometry
        # self.pose['x'] = self.xEst[0,0]
        # self.pose['y'] = self.xEst[1,0]
        # self.pose['th'] = self.xEst[2,0]
        # #
        # self.ready = True
        # print("AMCL EKF -----------------------------------------------------------------OK-----")

    def spin(self): #ok
        rospy.loginfo("[ROS][hiennd] Start NodeEKF_Publisher_Debug")
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
    # Matrix Wk [3x2] -------------------------------
    def jacob_fw(self,x, u,Ts):
        v       = u[0, 0]     # v - robot
        w       = u[1, 0]     # w - robot
        theta   = x[2,0]      # yaw
        #jW = 3x2
        jW = np.array([[Ts*math.cos(theta + 0.5*Ts*w), -0.5*v*(Ts**2)*math.sin(theta + 0.5*Ts*w)],
                    [ Ts*math.sin(theta + 0.5*Ts*w), -0.5*v*(Ts**2)*math.cos(theta + 0.5*Ts*w)],
                    [                            0,                                        Ts]])
        return jW
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
    def jacob_h_uwb_imu(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0],
            [0, 1, 0],
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
    #----------------EKF UWB-------------------------
    def ekf_estimation_uwb(self,xEst, PEst, z, u, Ts):
        # current_time = rospy.Time.now()
        # Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        # self.time_ekf_update = current_time                   #update time
        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + self.Q 
        #  Update
        jH = self.jacob_h_uwb()
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + self.R_UWB
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        xEst = xPred + K.dot((z - zPred)) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

        print("UWB: Kx= " + str(K[0,0]) + ", Ky=" + str(K[1,1]))
        return xEst, PEst
    #------------EKF UWB + IMU----------------------------
    def ekf_estimation_uwb_imu(self,xEst, PEst, z, u, Ts):
        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        #jW = self.jacob_fw(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + self.Q 

        #  Update
        jH = self.jacob_h_uwb_imu() #uwb + imu
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + self.R_UWB_IMU
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        xEst = xPred + K.dot((z - zPred)) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

        print("UWB: Kx= " + str(K[0,0]) + ", Ky=" + str(K[1,1]) + ", Kth=" + str(K[2,2]))
        return xEst, PEst
    #------------EKF IMU----------------------------
    def ekf_estimation_imu(self,xEst, PEst, z, u, Ts):
        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + self.Q

        #  Update    
        jH = self.jacob_h_imu()  #IMU ()
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + self.R_IMU
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        y = z - zPred
        xEst = xPred + K.dot(y) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

        #print("xPred=" + str(xPred[2,0]) + ", imu="+ str(z) + ", y=" + str(y) + ", K=" + str(K[2,0]) + ", theta=" + str(xEst[2,0]))

        return xEst, PEst   
    #-------EKF AMCL-----
    def ekf_estimation_amcl(self,xEst, PEst, z, u, Ts):
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        self.time_ekf_update = current_time                   #update time

        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + self.Q  

        #  Update    
        jH = self.jacob_h_amcl()  #IMU ()
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + self.R_AMCL
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        xEst = xPred + K.dot((z - zPred)) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

        print("AMCL: Kx= " + str(K[0,0]) + ", Ky=" + str(K[1,1]) + ", Kth=" + str(K[2,2]))

        return xEst, PEst   

# -------------Main--------------------
def main():
    print(msg)
    ekf = EKF_Publisher()
    ekf.spin()

if __name__ == '__main__':
    main()