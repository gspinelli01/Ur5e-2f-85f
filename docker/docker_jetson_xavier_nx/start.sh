#! /bin/bash
# Permission for teleoperator node
sudo chmod 777 /dev/hidraw0 
sudo chmod 777 /dev/input/event7

# Build Zed-Camera ros-package
source /opt/ros/noetic/setup.bash

cd /home/mivia/catkin_ws
sudo catkin build zed_camera_controller dataset_collector
source devel/setup.bash

# Open terminal
/bin/bash
