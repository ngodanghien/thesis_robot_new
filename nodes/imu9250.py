#!/usr/bin/python
from __future__ import division
# coding: utf-8

import rospy
import tf
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import Vector3, Vector3Stamped

import MPU9250
import math
from math import radians

# MAG_HARD_BIAS  =  (113.90899999999999, -40.54, -16.3515)
# MAG_SOFT_BIAS = (0.9602207761635727, 0.9829804630346844, 1.0624072704609615)

#MAG_HARD_BIAS  =  (132.1605, -30.133499999999998, -23.2225)
#MAG_SOFT_BIAS = (1.0291878517105106, 0.9204656212387662, 1.061623641331525)

G_TO_MS2 = 9.8

mpu9250 = MPU9250.MPU9250()

# shutdown ROS on interrupt
def shutdown():
    rospy.loginfo("Shutting Down Ploting")
    rospy.sleep(1)

# def rpyFilter(msg):
#     pass
#     yaw = Float32()
#     yaw.data = (msg.vector.z)*180/math.pi
#     imu_yaw_pub.publish(yaw)


try:
    rospy.init_node('node_imu_mpu9250', anonymous=True)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(1000)    #Hz
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    # mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
    # imu_yaw_pub = rospy.Publisher('imu/debug/yaw', Float32, queue_size=10)

    # rospy.Subscriber('/imu/rpy/filtered',   Vector3Stamped,    rpyFilter)

    imu_msg = Imu()
    mag_msg = MagneticField()
    rospy.loginfo("IMU STARTED")

    rospy.loginfo("IMU Calculator Bias -----------------------")

    bias_accel  = [0.0, 0.0, 0.0]
    bias_gyro   = [0.0, 0.0, 0.0]
    SAMPLE_COUNT = 2000
    i = 0
    #for i in range(SAMPLE_COUNT):
    while i <= SAMPLE_COUNT:
        if mpu9250.checkDataReady() == True:
            pass
            i += 1
            rospy.loginfo("IMU: runnung Calib Bias-----------------(" + str(i)+"/"+str(SAMPLE_COUNT)+").")
            acc = mpu9250.readAccel()
            gyr = mpu9250.readGyro()

            bias_accel[0] += acc[0]
            bias_accel[1] += acc[1]
            bias_accel[2] += acc[2]
            #
            bias_gyro[0] += gyr[0]
            bias_gyro[1] += gyr[1]
            bias_gyro[2] += gyr[2]
    # #
    bias_accel[0] /= SAMPLE_COUNT
    bias_accel[1] /= SAMPLE_COUNT
    bias_accel[2] /= SAMPLE_COUNT
    #
    bias_accel[2] = bias_accel[2] - 1.0

    # #
    bias_gyro[0] /= SAMPLE_COUNT
    bias_gyro[1] /= SAMPLE_COUNT
    bias_gyro[2] /= SAMPLE_COUNT

    bias_gyro = [radians(x) for x in bias_gyro] 

    rospy.loginfo("IMU Fisnish Calib Bias-------------------(OK)")

    rospy.loginfo("IMU: Bias (ax, ay, az) = " + str(bias_accel[0])  + ", " + str(bias_accel[1]) + ", " + str(bias_accel[2]))
    rospy.loginfo("IMU: Bias (gx, gy, gz) = " + str(bias_gyro[0])   + ", " + str(bias_gyro[1])  + ", " + str(bias_gyro[2]))

    while True and not rospy.is_shutdown():
        try:
            if mpu9250.checkDataReady() == True:
                pass
                m9a = mpu9250.readAccel()
                m9g = mpu9250.readGyro()
                mag = mpu9250.readMagnet()

                # m9a = [G_TO_MS2 * x for x in m9a]
                m9a = [x for x in m9a]
                m9g = [radians(x) for x in m9g]

                m9a[0] -= bias_accel[0]
                m9a[1] -= bias_accel[1]
                m9a[2] -= bias_accel[2]

                m9g[0] -= bias_gyro[0]
                m9g[1] -= bias_gyro[1]
                m9g[2] -= bias_gyro[2]
                
                # mx, my, mz = ((mag[x] - MAG_HARD_BIAS[x]) * MAG_SOFT_BIAS[x] for x in range(3))

                # Fill mag msg
                # mag_msg.header.stamp = rospy.get_rostime()
                # mag_msg.magnetic_field.x = mx
                # mag_msg.magnetic_field.y = my
                # mag_msg.magnetic_field.z = mz

                # create imu msg
                q0 = 1.0 #W
                q1 = 0.0 #X
                q2 = 0.0 #Y
                q3 = 0.0 #Z

                #Fill imu message
                imu_msg.header.stamp = rospy.get_rostime()
                # imu_msg.header.frame_id = 'imu_raw'
                imu_msg.header.frame_id = 'imu_link'

                imu_msg.orientation.x = q1
                imu_msg.orientation.y = q2
                imu_msg.orientation.z = q3
                imu_msg.orientation.w = q0
                imu_msg.orientation_covariance[0] = 1e6
                imu_msg.orientation_covariance[0] = 1e6
                imu_msg.orientation_covariance[0] = 0.1   

                imu_msg.angular_velocity.x = m9g[0]
                imu_msg.angular_velocity.y = m9g[1]
                imu_msg.angular_velocity.z = m9g[2]
                imu_msg.angular_velocity_covariance[0] = 1e6
                imu_msg.angular_velocity_covariance[4] = 1e6
                imu_msg.angular_velocity_covariance[8] = 0.1
                
                imu_msg.linear_acceleration.x = m9a[0]
                imu_msg.linear_acceleration.y = m9a[1]
                imu_msg.linear_acceleration.z = m9a[2]
                imu_msg.linear_acceleration_covariance[0] = 1e6
                imu_msg.linear_acceleration_covariance[4] = 1e6
                imu_msg.linear_acceleration_covariance[8] = 0.1

                imu_pub.publish(imu_msg)
                # mag_pub.publish(mag_msg)

                rate.sleep()
        except KeyboardInterrupt:
            break
except rospy.ROSInterruptException:
    rospy.logwarn("ROS_NODE_ENDED")
except Exception as e:
    rospy.logerr('IMU NODE EXCEPTION: ', e)