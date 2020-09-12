#!/bin/bash
cd ~/catkin_ws/
catkin_make &

sleep 1s

source /opt/ros/melodic/setup.bash &
source ~/catkin_ws/devel/setup.bash &