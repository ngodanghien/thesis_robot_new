#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 11 16:10:59 2020

@author: hiennd
Extended kalman filter (EKF) localization
"""


import math
import matplotlib.pyplot as plt
import numpy as np

#Thử nghiệm 01: Kết hợp với UWB
#Thử nghiệm 02: Kết hợp với IMU
#Thử nghiệm 03: Kết hợp với UWB + IMU
#---------------------------------------------
# Covariance for EKF simulation
Q = np.diag([
    0.1,            # variance of location on x-axis
    0.1,            # variance of location on y-axis
    np.deg2rad(30)  # variance of yaw angle - 0.017453292519943295
]) ** 2             # predict state covariance

#UWB
#R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance
#IMU
#R = 2
#UWB + IMU
R = np.diag([1.0, 1.0, 1.0]) ** 2 
#  Simulation parameter
INPUT_NOISE     = np.diag([1.0, np.deg2rad(30.0)]) ** 2
UWB_NOISE       = np.diag([0.5, 0.5]) ** 2    #[2x1]
IMU_NOISE       = np.deg2rad(10) ** 2         #[1x1]

UWB_IMU_NOISE   = np.diag([0.5, 0.5, np.deg2rad(5.0)]) ** 2
Ts = 0.1            # time tick [s] = 20ms
SIM_TIME = 50.0     # simulation time [s]

show_animation = True


def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v], [yawrate]])
    return u

# robot system model --------------
def robot_model_system(x, u):
    # Ref: (2.20) chapter (2.2.2)
    w = u[1,0]
    theta = x[2,0]
    # F = [3x3]
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])  
    # B = [3x2]
    B = np.array([[Ts*math.cos(theta + 0.5*Ts*w),    0],
                  [Ts*math.sin(theta + 0.5*Ts*w),    0],
                  [0.0,                             Ts]])
    # = [3x3][3x1] + [3x2][2x1] = [3x1]
    #x = F @ x + B @ u
    x = F.dot(x) + B.dot(u)

    #
    return x

# Matrix Fk [3x3] -------------------------------
def jacob_f(x, u):
    """
    Jacobian of Motion Model
    ref: (2.21) Chaper 2.2.2
    """ 
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
def jacob_fw(x, u):
    """
    Jacobian of Motion Model with w
    ref: (2.22) Chapter 2.2.2
    """ 
    v       = u[0, 0]     # v - robot
    w       = u[1, 0]     # w - robot
    theta   = x[2,0]      # yaw
    #jW = 3x2
    jW = np.array([[Ts*math.cos(theta + 0.5*Ts*w), -0.5*v*(Ts**2)*math.sin(theta + 0.5*Ts*w)],
                  [ Ts*math.sin(theta + 0.5*Ts*w), -0.5*v*(Ts**2)*math.cos(theta + 0.5*Ts*w)],
                  [                            0,                                        Ts]])
    return jW


#-------tao tin hieu quan sat gia UWB - xy (simulink) -----
def observation_uwb(xTrue, xd, u):
    xTrue = robot_model_system(xTrue, u)
    # add noise to uwb x-y
    # z = jacob_h_uwb() @ xTrue + UWB_NOISE @ np.random.randn(2, 1)
    z = jacob_h_uwb() * xTrue + UWB_NOISE * np.random.randn(2, 1)

    # add noise to input
    # ud = u + INPUT_NOISE @ np.random.randn(2, 1)
    ud = u + INPUT_NOISE * np.random.randn(2, 1)
    xd = robot_model_system(xd, ud)

    return xTrue, z, xd, ud

#-------tao tin hieu quan sat gia cho IMU - yaw (simulink) -----
global drift_yaw
def observation_imu(xTrue, xd, u):
    global drift_yaw
    
    xTrue = robot_model_system(xTrue, u)
    #add noise to yaw - imu
    # z = jacob_h_imu() @ xTrue + IMU_NOISE * np.random.randn(1, 1) + drift_yaw
    z = jacob_h_imu() * xTrue + IMU_NOISE * np.random.randn(1, 1) + drift_yaw
    drift_yaw += 0.001
    # add noise to input
    # ud = u + INPUT_NOISE @ np.random.randn(2, 1)
    ud = u + INPUT_NOISE * np.random.randn(2, 1)
    xd = robot_model_system(xd, ud)

    return xTrue, z, xd, ud
#-------tao tin hieu quan sat gia cho UWB + IMU -----
def observation_uwb_imu(xTrue, xd, u):
    xTrue = robot_model_system(xTrue, u)
    # add noise to uwb x-y
    # z = jacob_h_uwb_imu() @ xTrue + UWB_IMU_NOISE @ np.random.randn(3, 1)
    z = jacob_h_uwb_imu().dot(xTrue) + UWB_IMU_NOISE.dot(np.random.randn(3, 1))

    # add noise to input
    # ud = u + INPUT_NOISE @ np.random.randn(2, 1)
    ud = u + INPUT_NOISE.dot(np.random.randn(2, 1))

    xd = robot_model_system(xd, ud)

    return xTrue, z, xd, ud
# H(uwb) [2x3]---------------------------
def jacob_h_uwb():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0],
        [0, 1, 0] ])

    return jH
# H(IMU-Yaw) [1x3] -----------------------
def jacob_h_imu():
    # Jacobian of Observation Model
    jH = np.array([
        [0, 0, 1]])   
    return jH
# H(UWB-IMU) [3x3] -----------------------
def jacob_h_uwb_imu():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]])
    return jH

#----------------------------------------
def ekf_estimation(xEst, PEst, z, u):
    #  Predict
    xPred = robot_model_system(xEst, u)
    jF = jacob_f(xPred, u)
    # PPred = jF @ PEst @ jF.T + Q 
    PPred = jF.dot(PEst).dot(jF.T) + Q 
    #jW = jacob_fw(xPred, u)
    #PPred = jF @ PEst @ jF.T + jW @ Q1 @ jW.T

    #  Update
    #jH = jacob_h_uwb()  #TODO: UWB
    #jH = jacob_h_imu()  #IMU
    jH = jacob_h_uwb_imu()   #UWB+IMU   
    # zPred = jH @ xPred
    zPred = jH.dot(xPred)
    #y = z - zPred
    # S = jH @ PPred @ jH.T + R
    # K = PPred @ jH.T @ np.linalg.inv(S)
    # xEst = xPred + K @ (z - zPred) #
    # PEst = (np.eye(len(xEst)) - K @ jH) @ PPred

    S = jH.dot(PPred).dot(jH.T) + R
    K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
    xEst = xPred + K.dot((z - zPred)) #
    PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    rot = np.array([[math.cos(angle), math.sin(angle)],
                    [-math.sin(angle), math.cos(angle)]])
    # fx = rot @ (np.array([x, y]))
    fx = rot.dot((np.array([x, y])))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    print(__file__ + " start EKF Fustion Simulink !!")

    time = 0.0

    # State Vector [x y yaw]'
    xEst = np.zeros((3, 1))
    xTrue = np.zeros((3, 1))    #3x1
    PEst = np.eye(3)

    xDR = np.zeros((3, 1))      # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    #hz = np.zeros((2, 1))    #TODO: uwb
    #hz = np.zeros((1, 1))      #imu
    hz = np.zeros((3, 1))       #UWB+IMU
    global drift_yaw
    drift_yaw = 0.0

    while SIM_TIME >= time:
        time += Ts
        u = calc_input()
        
        #xTrue, z, xDR, ud = observation_uwb(xTrue, xDR, u)  #uwb
        #xTrue, z, xDR, ud = observation_imu(xTrue, xDR, u) #imu
        xTrue, z, xDR, ud = observation_uwb_imu(xTrue, xDR, u) #uwb+imu

        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))

    # if show_animation:
    #     plt.subplots(1,1)
    #     plt.cla()
    #     plt.plot(hz[0, :], hz[1, :], ".g")  #uwb
    #     plt.plot(hxTrue[0, :].flatten(),
    #               hxTrue[1, :].flatten(), "-b", label='xTrue')

    #     plt.plot(hxDR[0, :].flatten(),
    #               hxDR[1, :].flatten(), "-k", label='xDR')
    #     plt.plot(hxEst[0, :].flatten(),
    #               hxEst[1, :].flatten(), "-r", label='xEst')
    #     plot_covariance_ellipse(xEst, PEst)
    #     plt.legend(loc='upper right')
    #     plt.axis("equal")
    #     plt.grid(True)
    #     plt.pause(0.001)
        
    #     #yaw - theta
    #     plt.subplots(1,1)
    #     #plt.plot(np.rad2deg(hz[0,:].flatten()), ".g", label='Yaw')  #only IMU
    #     plt.plot(np.rad2deg(hz[2,:].flatten()), ".g", label='Yaw')  #uwb+imu
    #     plt.plot(np.rad2deg(hxTrue[2, :].flatten()), "-b",label='xTrue')
    #     plt.plot(np.rad2deg(hxDR[2, :].flatten()), "-k",label='xDR')
    #     plt.plot(np.rad2deg(hxEst[2, :].flatten()), "-r",label='xEst')
    #     plt.legend(loc='upper left')
        
    #     #plt.legend(loc='upper right')
    #     plt.axis("equal")
    #     plt.grid(True)
    #     plt.pause(0.001)
    
    
    RMSE_DR   = np.sqrt(np.mean((hxTrue - hxDR)**2))
    RMSE_UWB    = np.sqrt(np.mean((hxTrue - hxEst)**2))
    RMSE_IMU    = np.sqrt(np.mean((hxTrue - hxEst)**2))
    
    print("RMSE_DR  : = "    + str(RMSE_DR))
    print("RMSE_UWB : = "   + str(RMSE_UWB))
    
    
if __name__ == '__main__':
    main()
