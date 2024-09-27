# To use ROS with the conda environment 
Follow the instructions reported [here](https://github.com/RoboStack/ros-noetic)

# To run AI-Based Controller
```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.100 kinematics_config:="/home/ciccio/.ros/real_robot_calibration.yaml" use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR robot_description_file:="/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_description/launch/load_ur5e_2f_85.launch"
```

# 2. Run controller
```bash
roslaunch ur5e_2f_85_controller controller.launch 
```

```bash
roslaunch ur5e_2f_85_camera_table_moveit_config moveit_rviz.launch 
``` 

# 2.1. Run gripper driver
```bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /tmp/ttyUR
```
# 2.1. If you want to collect correct sample
```bash
roslaunch ur5e_2f_85_teleoperation ur5e_teleoperation.launch
```


# 2.2. Run camera controller
```bash
roslaunch zed_camera_controller zed_camera_controller.launch
```