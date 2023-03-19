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

roslaunch ur5e_2f_85_moveit_config moveit_planning_execution.launch moveit_controller_manager:="real"
roslaunch ur5e_moveit_config moveit_rviz.launch config:=true
```

``` bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.56.101 kinematics_config:=/home/ciccio/.ros/robot_calibration.yaml robot_description_file:=/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_description/launch/load_ur5e_2f_85.launch
```

