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
<!-- 
```bash
cd /home/gianl/catkin_ws/src/Ur5e-2f-85f
git pull
adjust the indentation
``` -->


# at startup
add `--write_summary` to `ai_controller_test.launch`
- select `cnt` variable in `test_node.py`

# if you want to import rt1 from multi_task_il.models
```bash
cd /home/gianl/Multi-Task-LFD-Framework/repo/Multi-Task-LFD-Training-Framework
git branch -v -a
git switch gspinelli # create a local copy of the remote branch
git pull

#for CTOD:
git switch ur5e
    

# 2.3 Run test node
# [REMEMBER] to do `xhost si:localuser:root` from gianl user
```bash
sudo -i
cd /home/gianl/catkin_ws
source devel/setup.bash
roslaunch ur5e_2f_85_controller ai_controller_test.launch
```

<!-- # optional
```bash
sudo -i
cd /home/gianl/catkin_ws
source devel/setup.bash
roslaunch ur5e_2f_85_camera_table_moveit_config moveit_rviz.launch 
```  -->



```
[REMEMBER] paste the test_node.py

# to switch to main branch
git checkout main

# to switch to gspinelli branch
git checkout gspinelli




