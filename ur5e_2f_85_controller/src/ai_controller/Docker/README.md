```bash
# Build the docker
export GID=$(id -g ciccio) && sudo docker build -t ai-controller --build-arg USER_UID=$UID --build-arg USERNAME=$USER --build-arg USER_GID=$GID --build-arg GIT_PERSONAL_TOKEN=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 .

sudo docker run -it --rm --privileged --name="ai-controller_container" --network="host" --runtime nvidia -v /dev:/dev -v /home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f:/catkin_ws/src/Ur5e-2f-85f  -v /media/ciccio/Sandisk/multitask_dataset_ur/multitask_dataset_language_command:/media/ciccio/Sandisk/multitask_dataset_ur/multitask_dataset_language_command -p 5678:5678 -e NVIDIA_DRIVER_CAPABILITIES=all ai-controller

# Update clock
```bash
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"
```


# To run AI-controller
1. Docker run
```bash
sudo docker run -it --rm --privileged --name="ai-controller_container" --network="host" --runtime nvidia -v /dev:/dev -v /tmp:/tmp -v /home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f:/catkin_ws/src/Ur5e-2f-85f -v /home/ciccio/Desktop/catkin_ws/src/ZED-Controller/zed_camera_controller:/catkin_ws/src/ZED-Controller/zed_camera_controller  -v /media/ciccio/Sandisk/multitask_dataset_ur/multitask_dataset_language_command:/media/ciccio/Sandisk/multitask_dataset_ur/multitask_dataset_language_command -p 5678:5678 -e NVIDIA_DRIVER_CAPABILITIES=all ai-controller
```
2. UR driver
```bash
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.1.102 target_filename:="/home/ciccio/.ros/real_robot_calibration.yaml"
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.102 kinematics_config:="/home/ciccio/.ros/real_robot_calibration.yaml" use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR robot_description_file:="/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_description/launch/load_ur5e_2f_85.launch"
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /tmp/ttyUR
```
3. Run dependencies
```bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /tmp/ttyUR
roslaunch zed_camera_controller zed_camera_controller.launch
roslaunch ur5e_2f_85_camera_table_moveit_config moveit_rviz.launch 
```
3. Run ai-controller
```
sudo su
roslaunch ur5e_2f_85_controller ai_controller.launch 
```