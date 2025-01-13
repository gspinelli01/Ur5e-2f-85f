u open
```bash
# you have to do this BEFORE executing roslaunch after
sudo -i
cd /home/gianl/catkin_ws
source devel/setup.bash
```

# 1 Run robot ros driver
# [REMEMBER] to press play on Ur5e screen
sudo -i
cd /home/gianl/catkin_ws
source devel/setup.bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.100 kinematics_config:="/home/docker_ubuntu20/robot_calibration.yaml" use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR robot_description_file:="/home/gianl/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_description/launch/load_ur5e_2f_85.launch"
```

# 2. Run controller
```bash
sudo -i
cd /home/gianl/catkin_ws
source devel/setup.bash
roslaunch ur5e_2f_85_controller controller.launch 
```

```bash
sudo -i
cd /home/gianl/catkin_ws
source devel/setup.bash
roslaunch ur5e_2f_85_camera_table_moveit_config moveit_rviz.launch 
``` 

# 2.1. Run gripper driver
```bash
sudo -i
cd /home/gianl/catkin_ws
source devel/setup.bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /tmp/ttyUR
```

# 2.2. Run camera controller
```bash
sudo -i
cd /home/gianl/catkin_ws
source devel/setup.bash
roslaunch zed_camera_controller zed_camera_controller.launch
```

# 2.3 Run test node
# [REMEMBER] to do `xhost si:localuser:root` from gianl user
```bash
sudo -i
cd /home/gianl/catkin_ws
source devel/setup.bash
roslaunch ur5e_2f_85_controller ai_controller_test.launch
```