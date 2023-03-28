```bash
# Build the docker
export GID=$(id -g mivia)
sudo docker build --ssh default=C:/Users/franc/.ssh/id_rsa -t jetson_xavier_nx --build-arg USER_UID=$UID --build-arg USERNAME=$USER --build-arg USER_GID=$GID --build-arg GIT_PERSONAL_TOKEN=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 .

# Update clock
```bash
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"
```

# Run the docker 
```
sudo docker run -it --network="host" --name ur_controller -v /home/mivia/Desktop/frosa/Ur5e-2f-85f/docker/docker_jetson_xavier_nx:/home/mivia/docker_jetson_xavier_nx --rm jetson_xavier_nx 
sudo docker exec ur_controller /bin/bash

```bash
# 1. Run calibration
roslaunch ur_calibration calibration_correction.launch robot_ip:=172.16.174.31 target_filename:="/home/mivia/docker_jetson_xavier_nx/robot_calibration.yaml"
# 2. Run bringup
# without tool communication
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=172.16.174.31 kinematics_config:=/home/mivia/docker_jetson_xavier_nx/robot_calibration.yaml 
# with tool communication
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=172.16.174.31 kinematics_config:=/home/mivia/docker_jetson_xavier_nx/robot_calibration.yaml use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR 
```