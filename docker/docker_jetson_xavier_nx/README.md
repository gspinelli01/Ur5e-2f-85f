```bash
# Build the docker
export GID=$(id -g mivia)
sudo docker build --ssh default=C:/Users/franc/.ssh/id_rsa -t jetson_xavier_nx --build-arg USER_UID=$UID --build-arg USERNAME=$USER --build-arg USER_GID=$GID --build-arg GIT_PERSONAL_TOKEN=<YOUR PERSONAL ACCESS TOKEN> .

# Update clock
```bash
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"
```

# Fix the problem with docker run
See this [link](https://github.com/dusty-nv/jetson-containers/issues/108) 

# Run the docker 
```bash
sudo docker run -it --gpus all --runtime=nvidia --rm --network="host" --privileged --name ur_controller -v /home/mivia/Desktop/frosa/Ur5e-2f-85f/docker/docker_jetson_xavier_nx:/home/mivia/docker_jetson_xavier_nx -v /dev:/dev jetson_xavier_nx 

sudo docker exec -it ur_controller /bin/bash /home/mivia/docker_jetson_xavier_nx/start.sh
```


```bash
# 1. Run calibration
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.1.102 target_filename:="/home/mivia/docker_jetson_xavier_nx/robot_calibration.yaml"
# 2. Run bringup
# without tool communication
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.102 kinematics_config:=/home/mivia/docker_jetson_xavier_nx/robot_calibration.yaml 
# with tool communication
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.102 kinematics_config:=/home/mivia/docker_jetson_xavier_nx/robot_calibration.yaml use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR 

# 3. Run teleoperation node
# Joyteleoperation permisission
sudo chmod 777 /dev/hidraw0 
sudo chmod 777 /dev/input/event7
roslaunch ur5e_2f_85_teleoperation ur5e_teleoperation.launch
```