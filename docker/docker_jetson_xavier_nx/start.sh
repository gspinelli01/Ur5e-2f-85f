#! /bin/bash
echo "sourcing ros"
source /opt/ros/noetic/setup.bash
echo "sourcing workspace"
source devel/setup.bash

# Permission for teleoperator node
sudo chmod 777 /dev/hidraw0 
sudo chmod 777 /dev/input/event7

# Open terminal
/bin/bash