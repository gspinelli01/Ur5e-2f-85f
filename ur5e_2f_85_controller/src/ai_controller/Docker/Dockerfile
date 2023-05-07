FROM nvidia/cuda:11.7.0-devel-ubuntu20.04

SHELL ["/bin/bash", "-c"]
# System packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && apt-get install -y \
    python3.8 \
    wget \
    git \
    bash-completion \
    build-essential \
    sudo \
    lsb-core \
    curl
RUN rm -rf /var/lib/apt/lists/*

# Install ROS
# Install ROS noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt update -y
RUN apt install -y ros-noetic-ros-base
RUN echo "source /opt/ros/noetic/setup.bash" >>~/.bashrc
RUN source ~/.bashrc
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN apt dist-upgrade -y
RUN apt-get -y install git cmake ros-noetic-catkin python3-catkin-tools python3-setuptools libgtest-dev build-essential
RUN source /opt/ros/noetic/setup.bash
RUN rm -rf /var/lib/apt/lists/*
RUN rosdep init
RUN rosdep update

# Now create the same user as the host itself
# Add user
ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG GIT_PERSONAL_TOKEN

# Create the user
RUN groupadd --gid $USER_GID $USERNAME
RUN useradd --uid $USER_UID --gid $USER_GID -m $USERNAME
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL >/etc/sudoers.d/$USERNAME
RUN chmod 0440 /etc/sudoers.d/$USERNAME

# Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
ENV PATH=/home/$USERNAME/.local/bin:$PATH

# COPY Workspace
WORKDIR ${HOME}/catkin_ws/src
RUN ["cp", "-r", "/home/ciccio/Desktop/catkin_ws", "${HOME}"]

# Install Moveit
WORKDIR ${HOME}/catkin_ws/src/moveit
RUN sudo wstool init .
RUN sudo wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
RUN sudo wstool remove moveit_tutorials
RUN rosdep update
RUN sudo apt-get update -y
RUN sudo wstool update -t . && rosdep install -y --from-paths . --ignore-src --rosdistro noetic --os=ubuntu:focal

# Install ZED-Drivers
WORKDIR /home/temp
RUN sudo apt install -y zstd
RUN sudo wget -q --no-check-certificate https://download.stereolabs.com/zedsdk/4.0/cu118/ubuntu20
RUN sudo chmod +x ubuntu20
RUN sudo ./ubuntu20 silent skip_tools && sudo rm -rf /usr/local/ubuntu20/resources/* && sudo rm -rf ubuntu20 && sudo rm -rf /var/lib/apt/lists/*

# Build catkin workspace
WORKDIR ${HOME}/catkin_ws/

# 1. UR-Driver
RUN rosdep install -y --from-paths ./src/Universal_Robots* --ignore-src --rosdistro noetic --os=ubuntu:focal

# 2. Gripper dependencies
RUN rosdep install -y --from-paths ./src/robotiq --ignore-src --rosdistro noetic --os=ubuntu:focal
RUN rosdep install --from-paths . --ignore-src -y
RUN sudo apt-get install ros-noetic-soem
WORKDIR ${HOME}/catkin_ws
RUN catkin config --skiplist robotiq_2f_140_gripper_visualization robotiq_2f_c2_gripper_visualization robotiq_3f_gripper_articulated_msgs robotiq_3f_gripper_articulated_gazebo robotiq_3f_gripper_articulated_gazebo_plugins robotiq_3f_gripper_control robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_visualization robotiq_3f_rviz robotiq_ft_sensor
