#!/usr/bin/python
# Author: hiennd
# Date: 07/08/2020
# Test: OK

msg = """
    Author: hiennd
    Create date:     07/08/2020
    Test: PASSED OK (07/08/2020)
    -----------------------------------
    Node: 
        * /node_navigation_test
    Published topics:
        * /cmd_vel   [geometry_msgs/Twist]
    ------------------------------------
    How to run:
    #Circle
        $ rosrun thesis_robot_new nav_all.py    
    #Square
        $ rosrun thesis_robot_new nav_all.py _mode_run:='square'
    #Num8
        $ rosrun thesis_robot_new nav_all.py _mode_run:='num8'
    --------------------------------------
"""

import rospy
import math
from geometry_msgs.msg import Twist

class Run_Nav:
    def __init__(self):
        pass
        #Node
        rospy.init_node('node_navigation_test', anonymous=True)
        
        #Pub
        self.rate = 50     #Hz
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=self.rate)
        self.time_start = rospy.Time.now()
        
        self.mode_run = rospy.get_param('~mode_run','circle')    #circle|square|num8
            
    def spin(self): #ok
        rospy.loginfo("[ROS][hiennd] Start Navigation Test")
        self.update()
    
    def shutdown(self):
        rospy.loginfo("Stop navigation sim")
        # Stop message
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
    def update(self):
        pass
        #check mode run ...
        if self.mode_run == "circle": #circle|square|num8
            self.run_circle()
        #
        if self.mode_run == "square": #circle|square|num8
            self.run_square()
        #
        if self.mode_run == "num8": #circle|square|num8
            self.run_num8()
    
    def run_circle(self):
        pass
        rospy.loginfo("[ROS][hiennd] Start Navigation Test: One Circle")
        rate = rospy.Rate(200)     #200Hz = 5ms
        rospy.on_shutdown(self.shutdown)
        #
        v = 0.5
        w = 0.5
        time_start = rospy.Time.now()
        timeOneCircle = (2*math.pi/w)   #s
        rospy.loginfo('[ROS][hiennd] timeOneCircle = ' + str(timeOneCircle))
        #
        move_cmd = Twist()
        move_cmd.linear.x   = v     # v = 0.5 m/s
        move_cmd.angular.z  = w    # w = 0.5 rad/s
 
        # cho truyen toc do cham thoi .... ~ 100ms 
        i  = 0 # 20 * 5ms = 100ms
        #
        while not rospy.is_shutdown():
            time_duration = (rospy.Time.now() - time_start).to_sec()
            # code here
            i += 1
            if i >= 20: #~ 100ms = 10Hz
                i = 0
                self.cmd_vel_pub.publish(move_cmd)
            #
            if time_duration > timeOneCircle:
                rospy.signal_shutdown('[ROS][hiennd] Done node_nav_Circle') #PASSED
            #
            rospy.loginfo('[ROS][hiennd] time duration = ' + str(time_duration))
            rate.sleep()   

    def run_square(self):
        pass
        rospy.loginfo("[ROS][hiennd] Start Navigation Test: One Circle")
        rate = rospy.Rate(200)     #200Hz = 5ms
        rospy.on_shutdown(self.shutdown)
        #
        v = 0.5
        w = 0.785398163397448
        timeOneSquare = 24  #s

        time_start = rospy.Time.now()
        rospy.loginfo('[ROS][hiennd] timeOneSquare = ' + str(timeOneSquare))
        #
        move_cmd = Twist()
        move_cmd.linear.x   = v     # v = 0.5 m/s
        move_cmd.angular.z  = w    # w = 0.5 rad/s
 
        # cho truyen toc do cham thoi .... ~ 100ms 
        i  = 0 # 20 * 5ms = 100ms
        #
        while not rospy.is_shutdown():
            time_duration = (rospy.Time.now() - time_start).to_sec()
            # code here
            i += 1
            if i >= 20: #~ 100ms = 10Hz
                i = 0
                self.cmd_vel_pub.publish(move_cmd)
            # calc
            if time_duration < 4:
                pass
                v = 0.5
                w = 0.0
            if time_duration >= 4 and time_duration < 6:
                pass
                v = 0.0
                w = 0.785398163397448
            if time_duration >= 6 and time_duration < 10:
                pass
                v = 0.5
                w = 0.0
            if time_duration >= 10 and time_duration < 12:
                pass
                v = 0.0
                w = 0.785398163397448
            if time_duration >= 12 and time_duration < 16:
                pass
                v = 0.5
                w = 0.0
            if time_duration >= 16 and time_duration < 18:
                pass
                v = 0.0
                w = 0.785398163397448
            if time_duration >= 18 and time_duration < 22:
                pass
                v = 0.5
                w = 0.0
            if time_duration >= 22:
                pass
                v = w = 0
            #set v, w
            move_cmd.linear.x   = v     # v = m/s
            move_cmd.angular.z  = w     # w = rad/s
            #
            if time_duration > timeOneSquare:
                rospy.signal_shutdown('[ROS][hiennd] Done node_nav_Circle') #PASSED
            #
            rospy.loginfo('[ROS][hiennd] time duration = ' + str(time_duration))
            rate.sleep()   

    def run_num8(self):
        pass
        rospy.loginfo("[ROS][hiennd] Start Navigation Test: One Circle")
        rate = rospy.Rate(200)     #200Hz = 5ms
        rospy.on_shutdown(self.shutdown)
        #
        v = v_ = 0.5
        w = w_ = 0.5
        timeOneNum8 = 28  #s
        time_start = rospy.Time.now()
        rospy.loginfo('[ROS][hiennd] timeOneNum8 = ' + str(timeOneNum8))
        #
        move_cmd = Twist()
        move_cmd.linear.x   = v     # v = 0.5 m/s
        move_cmd.angular.z  = w    # w = 0.5 rad/s
 
        # cho truyen toc do cham thoi .... ~ 100ms 
        i  = 0 # 20 * 5ms = 100ms
        #
        while not rospy.is_shutdown():
            time_duration = (rospy.Time.now() - time_start).to_sec()
            # code here
            i += 1
            if i >= 20: #~ 100ms = 10Hz
                i = 0
                self.cmd_vel_pub.publish(move_cmd)
            # calc
            # calc
            if time_duration < 6.283:
                pass
                v = v_
                w = w_
            if time_duration >= 6.283 and time_duration < 18.85:
                pass
                v = v_
                w = -w_
            if time_duration >= 18.85 and time_duration < 25:
                pass
                v = v_
                w = w_
            if time_duration >= 25:
                v = w = 0.0
            #
            #set v, w
            move_cmd.linear.x   = v     # v = m/s
            move_cmd.angular.z  = w     # w = rad/s
            #
            if time_duration > timeOneNum8:
                rospy.signal_shutdown('[ROS][hiennd] Done node_nav_Circle') #PASSED
            #
            rospy.loginfo('[ROS][hiennd] time duration = ' + str(time_duration))
            rate.sleep()   

        




# -------------Main--------------------
def main():
    print(msg)
    nav = Run_Nav()
    nav.spin()
    #nav.run_circle()
    #nav.run_square()
    #nav.run_num8()

if __name__ == '__main__':
    main()
    