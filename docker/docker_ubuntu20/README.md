
# if you don't use devcontainers (if you do, skip to 'Before starting container')
## Build the docker

```bash
# first build the image in ros_noetic_v1 folder
cd ros_noetic_v1
docker build -t ros_noetic:v1 .

# then build the docker from the first image
cd ..
docker build -t ros_ur5e_noetic:v1 --build-arg GIT_PERSONAL_TOKEN=<YOUR_PERSONAL_GIT_TOKEN> .
```

## Run the docker 
```bash
sudo docker run -it --gpus all --runtime=nvidia --rm --network="host" -p 2222:22 --privileged --name ur_controller -v /home/gianl/uni/Ur5e-2f-85f/docker/docker_ubuntu20:/home/docker_ubuntu20 -v /dev:/dev --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ros_ur5e_noetic:v1

sudo docker exec -it ur_controller /bin/bash
```

# Before starting container
* make sure the SSD is already connected to pc

# Once started the container
* make sure the SSD is visible inside container
* do the following changes:
```bash
#1) /home/gianl/catkin_ws/src/Learning-from-Demonstration-Dataset-Collector/dataset_collector/config/dataset_collector.yaml
/media/ciccio -> /media/gianl

######################################################
#   1.a) ALSO CHECK TASK_ID AND START_COUNTER!!!!!!  #
######################################################

#2) camera_controller -> zed_camera_controller line 13 of /home/gianl/catkin_ws/src/Learning-from-Demonstration-Dataset-Collector/dataset_collector/src/trajectory_collector.py

#3) change the file `/home/gianl/catkin_ws/src/Learning-from-Demonstration-Dataset-Collector/dataset_collector/scripts/add_bb.py` with the one saved on pc `/home/gianl/save.py`

#4) sudo python3 -m pip install debugpy
```

# for every shell you open
```bash
# you have to do this BEFORE executing roslaunch after
sudo -i
cd /home/gianl/catkin_ws
source devel/setup.bash
```

# roslaunches (for every one of them, open a new shell)
```bash
# only at first boot
roslaunch ur_calibration calibration_correction.launch robot_ip:=172.16.174.10 target_filename:="/home/docker_ubuntu20/robot_calibration.yaml"

# Run robot ros driver
# [REMEMBER] to press play on Ur5e screen
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=172.16.174.10 kinematics_config:="/home/docker_ubuntu20/robot_calibration.yaml" use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR robot_description_file:="/home/gianl/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_description/launch/load_ur5e_2f_85.launch"

# Run moveit interface 
roslaunch ur5e_2f_85_controller controller.launch 

# Run dataset teleoperation node
roslaunch ur5e_2f_85_teleoperation ur5e_teleoperation.launch 

# Camera 
roslaunch zed_camera_controller zed_camera_controller.launch 

# Run dataset collector
# [REMEMBER] to do `xhost si:localuser:root` from gianl user
roslaunch dataset_collector dataset_collector.launch 
```