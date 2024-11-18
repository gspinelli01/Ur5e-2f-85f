#! /bin/bash
# Permission for teleoperator node
sudo chmod 777 /dev/hidraw0 
sudo chmod 777 /dev/input/event7

# Build Zed-Camera ros-package
source /opt/ros/noetic/setup.bash

sudo chown -R gianl:gianl /usr/local/zed/lib

cd $HOME
echo "export PATH='/usr/local/cuda-11.7/bin:$PATH'" >> .bashrc && echo "export LD_LIBRARY_PATH='/usr/local/zed/lib:/usr/local/cuda-11.7/lib64:$LD_LIBRARY_PATH'" >> .bashrc
source .bashrc

# Open terminal
/bin/bash

