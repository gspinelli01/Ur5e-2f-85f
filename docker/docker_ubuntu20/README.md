# Run the docker 
```bash
sudo docker run -it --gpus all --runtime=nvidia --rm --network="host" --privileged --name ur_controller -v /home/gianl/uni/Ur5e-2f-85f/docker/docker_ubuntu20:/home/docker_ubuntu20 -v /dev:/dev --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ros_ur5e_noetic:v1

sudo docker exec -it ur_controller /bin/bash
```

```bash
# only at first boot
roslaunch ur_calibration calibration_correction.launch robot_ip:=172.16.174.10 target_filename:="/home/docker_ubuntu20/robot_calibration.yaml"

# Run robot ros driver
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=172.16.174.10 kinematics_config:="/home/docker_ubuntu20/robot_calibration.yaml" use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR robot_description_file:="/home/gianl/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_description/launch/load_ur5e_2f_85.launch"

# Run moveit interface 
roslaunch ur5e_2f_85_controller controller.launch 

# Camera 
roslaunch zed_camera_controller zed_camera_controller.launch 

# Run dataset teleoperation node
roslaunch ur5e_2f_85_teleoperation ur5e_teleoperation.launch 

# Run dataset collector
roslaunch dataset_collector dataset_collector.launch 

```
