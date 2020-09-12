#!/usr/bin/python

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
from geometry_msgs.msg import Point, Quaternion, Twist, PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


# Covariance for EKF simulation
Q = np.diag([
    0.2,            # variance of location on x-axis
    0.2,            # variance of location on y-axis
    np.deg2rad(30)  # variance of yaw angle - 0.017453292519943295
]) ** 2             # predict state covariance

R_UWB              = np.diag([0.1, 0.1]) ** 2  # Observation x,y position covariance
R_IMU              = 0.1

show_animation = True

Ts = 0.1            # time tick [s] = 20ms
SIM_TIME = 50.0     # simulation time [s]

#--------------------------------------

class EKF_Publisher:
    def __init__(self):
        rospy.init_node('Node_EKF_Publisher', anonymous=True)
        #Pub
        self.ekf_pose_debug                 = rospy.Publisher("/debug/ekf_pose_debug", Point, queue_size=50)    #50Hz
        # self.yaw_imu_drift_debug            = rospy.Publisher("/debug/yaw_imu_sub_drift", Float32, queue_size=self.rate)
        self.amcl_init                      = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)    #50Hz
        #initialpose [geometry_msgs/PoseWithCovarianceStamped] 1 publisher
        #Sub
        rospy.Subscriber('/robot/vel_pub',      Twist,      self.vel_callback)
        rospy.Subscriber('/dwm1001c/tag',       Point,      self.uwb_sub_callback)
        rospy.Subscriber('/debug/yaw',          Point,      self.yaw_sub_callback)

        self.pose = {'x':0.0, 'y': 0.0, 'th': 0.0}
        self.vel = {'v':0.0 , 'w':0.0} #v robot (m/s)  && w robot (rad/s)

        #for debug
        self.time_uwb = rospy.Time.now()
        self.time_yaw = rospy.Time.now()
        self.time_vel = rospy.Time.now()

        #for predict EKF
        self.time_ekf_update = rospy.Time.now()
        self.PEst = np.eye(3)*10 
        self.xEst = np.zeros((3, 1))
        self.xEst[0] = 2.19
        self.xEst[1] = 5.07
        
        self.ready = False
        #
        self.timeCount = 0.0

        #
        # store data history
        self.hxEst = self.xEst
        # self.hxTrue = self.xTrue
        # self.hxDR = self.xTrue

        # hxEst = np.hstack((hxEst, xEst))
        # hxDR = np.hstack((hxDR, xDR))
        # hxTrue = np.hstack((hxTrue, xTrue))
        # hz = np.hstack((hz, z))


    def update(self):
        pass
        if self.ready == True:
            self.pub_ekf_debug()

        # self.timeCount += 1 #Ts

        # if (self.timeCount > 1000):#4750):
        #     self.draw()
        #print("count = " + str(self.timeCount))

    def pub_ekf_debug(self):
        pose_ = Point()
        pose_.x = self.xEst[0]
        pose_.y = self.xEst[1]
        pose_.z = self.xEst[2]
        self.ekf_pose_debug.publish(pose_)
    def pub_amcl_init(self):
        pass
        init_amcl = PoseWithCovarianceStamped()
        
        #amcl_init
    def vel_callback(self,vel_msg):
        pass
        self.vel['v'] = vel_msg.linear.x    # v robot (m/s)
        self.vel['w'] = vel_msg.angular.z   # w robot (rad/s)
        # current_time = rospy.Time.now()
        # hz = (current_time - self.time_vel).to_sec()   # ko deu ..50Hz nhung rat lon xon.
        # self.time_vel = current_time                   #
        # print("Time Vel Hz: " + str(hz) + " sencond (s)")
    
    def yaw_sub_callback(self, yaw_msg):
        pass
        self.ready = True
        
        u = np.array([[self.vel['v']], [self.vel['w']]])

        tmp = yaw_msg.y
        z = np.array([[tmp]])

        self.xEst, self.PEst = self.ekf_estimation_imu(self.xEst, self.PEst, z, u)
        

    def uwb_sub_callback(self,uwb_msg):
        self.ready = True
        pass
        # current_time = rospy.Time.now()
        # hz = (current_time - self.time_uwb).to_sec()   # 500ms
        # self.time_uwb = current_time                   #
        # print("Time UWB Hz: " + str(hz) + " sencond (s)")
        pass
        #self.xy_uwb_tag['x'] = uwb_msg.x
        #self.xy_uwb_tag['y'] = uwb_msg.y
        #rospy.loginfo("[DEBUG][uwb_sub_callback]---------------: " + str(self.xy_uwb_tag['x']) + ", " + str(self.xy_uwb_tag['y'])) #PASSED
        #cu co data la update
        u = np.array([[self.vel['v']], [self.vel['w']]])
        z = np.array([[uwb_msg.x],[uwb_msg.y]])

        #TODO:
        self.xEst, self.PEst = self.ekf_estimation_uwb(self.xEst, self.PEst, z, u) #PASSED: 100ms = 10Hz
        #history
        #self.hxEst = np.hstack((self.hxEst, self.xEst))

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


        # rospy.loginfo("[DEBUG][ekf_estimation_uwb]---------------: " + str(xEst)) 

        # Update to Odometry
        self.pose['x'] = xEst[0,0]
        self.pose['y'] = xEst[1,0]
        self.pose['th'] = xEst[2,0]

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
        S = jH.dot(PPred).dot(jH.T) + R_IMU
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        xEst = xPred + K.dot((z - zPred)) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

        return xEst, PEst   
    def draw(self):
        if show_animation:
            plt.subplots(1,1)
            plt.cla()
            # plt.plot(hz[0, :], hz[1, :], ".g")  #uwb
            # plt.plot(hxTrue[0, :].flatten(),
            #         hxTrue[1, :].flatten(), "-b", label='xTrue')

            # plt.plot(hxDR[0, :].flatten(),
            #         hxDR[1, :].flatten(), "-k", label='xDR')
            plt.plot(hxEst[0, :].flatten(),
                    hxEst[1, :].flatten(), "-r", label='xEst')
            # plot_covariance_ellipse(xEst, PEst)
            plt.legend(loc='upper right')
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)
            plt.show()
            
            #yaw - theta
            # plt.subplots(1,1)
            # #plt.plot(np.rad2deg(hz[0,:].flatten()), ".g", label='Yaw')  #only IMU
            # plt.plot(np.rad2deg(hz[2,:].flatten()), ".g", label='Yaw')  #uwb+imu
            # plt.plot(np.rad2deg(hxTrue[2, :].flatten()), "-b",label='xTrue')
            # plt.plot(np.rad2deg(hxDR[2, :].flatten()), "-k",label='xDR')
            # plt.plot(np.rad2deg(hxEst[2, :].flatten()), "-r",label='xEst')
            # plt.legend(loc='upper left')
            
            # #plt.legend(loc='upper right')
            # plt.axis("equal")
            # plt.grid(True)
            # plt.pause(0.001)
# -------------Main--------------------
def main():
    print(msg)
    ekf = EKF_Publisher()
    ekf.spin()

if __name__ == '__main__':
    main()