# Terminal 1
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

# Terminal 2
``` bash
# Run bash 
docker exec -it ursim_container /bin/bash

```

# Terminal 3
``` bash

roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.56.101 target_filename:="/home/ciccio/.ros/robot_calibration.yaml"

roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.56.101 kinematics_config:=/home/ciccio/.ros/robot_calibration.yaml

```

# Terminal 4
``` bash

roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch
roslaunch ur5e_moveit_config moveit_rviz.launch config:=true
```

``` bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=172.16.174.31 kinematics_config:=/home/ciccio/.ros/robot_calibration.yaml use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR 
```

