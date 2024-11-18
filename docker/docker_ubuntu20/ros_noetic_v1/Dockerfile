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