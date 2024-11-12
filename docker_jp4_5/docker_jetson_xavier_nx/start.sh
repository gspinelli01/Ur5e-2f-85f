#! /bin/bash
echo "sourcing ros"
source /opt/ros/noetic/setup.bash
echo "sourcing workspace"
source devel/setup.bash

# Permission for teleoperator node
sudo chmod 777 /dev/hidraw0 
sudo chmod 777 /dev/input/event7

# Build Zed-Camera ros-package
cd /home/mivia/catkin_ws
sudo catkin build zed_camera_controller
sudo source devel/setup.bash

# Open terminal
/bin/bash