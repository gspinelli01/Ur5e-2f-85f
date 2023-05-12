#! /bin/bash
echo "sourcing ros"
source /opt/ros/noetic/setup.bash
echo "sourcing workspace"
source /catkin_ws/devel/setup.bash

# # Permission for teleoperator node
sudo chmod 777 /dev/hidraw0
sudo chmod 777 /dev/input/event7

# # Build Zed-Camera ros-package
cd /catkin_ws
sudo catkin config --cmake-args -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined" ..
sudo catkin build zed_camera_controller
source devel/setup.bash

# Install mosaic
sudo python3 -m pip install /Multi-Task-LFD-Framework/repo/mosaic/.
sudo python3 -m pip install /Multi-Task-LFD-Framework/repo/Multi-Task-LFD-Training-Framework/training/.
# Open terminal
/bin/bash
sudo su
