# UR5e-2f-85


# Dependencies

## Clone repositories
1. UR-drivers repositories
```bash
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
git clone -b boost  https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
git clone https://github.com/ros-industrial/universal_robot.git
```
2. Custom modules
```bash
git clone git@github.com:ciccio42/robotiq.git
git clone git@github.com:ciccio42/Realsense-Camera-Controller.git
git clone git@github.com:ciccio42/Ur5e-85f.git
```
3. Joystick
```bash
git clone https://github.com/ros-drivers/joystick_drivers.git
```

## Installation
1. Robotiq dependencies.
```bash
rosdep install --from-paths src --ignore-src -y
sudo apt-get install ros-noetic-soem
```

2. Realsense installation. Follow instructions in Realsense-Camera-Controller README.md

3. Joystick. Follow the instructions reported [here](http://wiki.ros.org/joy)


## Compile
1. Blacklist for useless robotiq packages
```bash
catkin config --skiplist robotiq_2f_140_gripper_visualization robotiq_2f_c2_gripper_visualization robotiq_3f_gripper_articulated_msgs robotiq_3f_gripper_articulated_gazebo robotiq_3f_gripper_articulated_gazebo_plugins robotiq_3f_gripper_control robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_visualization robotiq_3f_rviz robotiq_ft_sensor 
```
2. Build
```bash
catkin build
```

# How to run

## Real-robot
1. Run external controller on robot
2. Run ur5e_bringup
```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=172.16.174.31 kinematics_config:=/home/ciccio/.ros/robot_calibration.yaml use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR 
```

## Simulation
1. Docker

``` bash
# Stop containers
docker stop ursim_container

# Remove previously docker with the same name
docker image rm ursim_docker

# Build Docker
docker build -t ursim_docker .

# Create docker subnetwork
docker network create --subnet=192.168.56.0/24 ursim_net

# Run Docker
docker run --rm --name ursim_container -it --net ursim_net --ip 192.168.56.101 -e ROBOT_MODEL=UR5 ursim_docker

```

``` bash

roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.56.101 target_filename:="/home/ciccio/.ros/robot_calibration.yaml"

roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.56.101 kinematics_config:=/home/ciccio/.ros/robot_calibration.yaml

```