FROM osrf/ros:noetic-desktop-full

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

RUN apt update -y && apt upgrade -y

# Install Git
RUN apt-get update -y && apt-get install -y git

# wget
RUN apt install wget

# catkin tools
RUN sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
        
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get install -y python3-catkin-tools

# ARG USER=user
# ARG DEBIAN_FRONTEND=noninteractive
# RUN apt-get install -y ros-noetic-moveit ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-rosserial ros-noetic-rosserial-arduino ros-noetic-roboticsgroup-upatras-gazebo-plugins ros-noetic-actionlib-tools terminator python3-pip && rm -rf /var/lib/apt/lists/*
# RUN pip install flask flask-ask-sdk ask-sdk
# WORKDIR /home/${USER}
# WORKDIR /home/gianl/uni/borsa_cogrob/MultiEngagement-ARI/ros_ws


# Change HOME environment variable
# ENV HOME=/.
# workspace setup
# RUN mkdir -p ${HOME}/catkin_ws/src
# RUN rosdep update

# Clone and build UR-drivers and custom repositories
# WORKDIR ${HOME}/catkin_ws/src
# WORKDIR ${HOME}/catkin_ws/src
# RUN sudo apt-get remove -y ros-noetic-ros-control

### non vengono installati
# RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
# RUN git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
# RUN git clone https://github.com/ros-industrial/universal_robot.git
# RUN rosdep install -y --from-paths . --ignore-src --rosdistro noetic --os=ubuntu:focal
