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

# Install Moveit
WORKDIR ${HOME}/catkin_ws/src/moveit
RUN sudo wstool init .
RUN sudo wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
RUN sudo wstool remove moveit_tutorials
RUN rosdep update
RUN sudo apt-get update -y
RUN sudo wstool update -t . && rosdep install -y --from-paths . --ignore-src --rosdistro noetic --os=ubuntu:focal

# Init workspace
WORKDIR ${HOME}/catkin_ws
RUN catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /opt/ros/noetic/setup.bash && catkin init
RUN source /opt/ros/noetic/setup.bash && sudo catkin build
RUN source devel/setup.bash

# 1. Clone and build UR-drivers and custom repositories
WORKDIR ${HOME}/catkin_ws/src
RUN sudo apt-get remove -y ros-noetic-ros-control
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
RUN git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
RUN git clone https://github.com/ros-industrial/universal_robot.git
RUN rosdep install -y --from-paths . --ignore-src --rosdistro noetic --os=ubuntu:focal
WORKDIR ${HOME}/catkin_ws
RUN sudo catkin build
RUN source devel/setup.bash

# 2. Robotiq repository
RUN git config --global http.sslverify false
WORKDIR ${HOME}/catkin_ws/src
RUN git clone -b noetic_2f_85 https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/robotiq.git
RUN rosdep install --from-paths . --ignore-src --rosdistro noetic --os=ubuntu:focal -y
RUN sudo apt-get install ros-noetic-soem
WORKDIR ${HOME}/catkin_ws
RUN catkin config --skiplist robotiq_2f_140_gripper_visualization robotiq_2f_c2_gripper_visualization robotiq_3f_gripper_articulated_msgs robotiq_3f_gripper_articulated_gazebo robotiq_3f_gripper_articulated_gazebo_plugins robotiq_3f_gripper_control robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_visualization robotiq_3f_rviz robotiq_ft_sensor
RUN sudo catkin build
RUN source devel/setup.bash

# 3. Joy dependencies
WORKDIR ${HOME}/catkin_ws/src
RUN mkdir ps4_joy_drivers
WORKDIR ${HOME}/catkin_ws/src/ps4_joy_drivers
RUN git clone https://github.com/chrippa/ds4drv.git
WORKDIR ${HOME}/catkin_ws/src/ps4_joy_drivers/ds4drv
RUN sudo apt-get update -y && sudo apt-get install -y python3-pip udev
RUN python3 -m pip install --upgrade pip
RUN sudo python3 setup.py install
COPY 50-ds4drv.rules /etc/udev/rules.d/50-ds4drv.rules
RUN sudo /lib/systemd/systemd-udevd --daemon
RUN sudo /lib/systemd/systemd-udevd --daemon && sudo udevadm control --reload-rules && sudo udevadm trigger
RUN git clone --branch noetic-devel https://github.com/naoki-mizuno/ds4_driver.git
WORKDIR ${HOME}/catkin_ws
RUN sudo catkin build

# 4. Install ZED-Drivers
WORKDIR /home/temp
RUN sudo apt install -y zstd
RUN sudo wget -q --no-check-certificate https://download.stereolabs.com/zedsdk/4.0/cu118/ubuntu20
RUN sudo chmod +x ubuntu20
RUN sudo ./ubuntu20 silent skip_tools && sudo rm -rf /usr/local/ubuntu20/resources/* && sudo rm -rf ubuntu20 && sudo rm -rf /var/lib/apt/lists/*

# 5. Install custom packages
WORKDIR ${HOME}/catkin_ws/src
RUN git clone https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/Ur5e-2f-85f.git
WORKDIR ${HOME}/catkin_ws
RUN sudo catkin build
WORKDIR ${HOME}/catkin_ws/src
RUN git clone https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/ZED-Controller.git

# 6. Install AI-modules (MOSAIC and Multi-Task IL framework)
WORKDIR ${HOME}
# 1. Multi-Task IL Framework
RUN git clone https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/Multi-Task-LFD-Framework.git
WORKDIR ${HOME}/Multi-Task-LFD-Framework/repo
# 1.1 robosuite
RUN git clone -b ur5e_ik https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/robosuite.git
# 1.2 mosaic
RUN git clone -b training-baseline https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/mosaic.git
# 1.3 Multi-Task-IL-Training-Framework
RUN git clone https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/Multi-Task-LFD-Training-Framework.git
# 1.4 Mujoco
# Mujoco set
ENV LANG C.UTF-8
RUN sudo mkdir -p ${HOME}/.mujoco/
RUN sudo wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz -O mujoco.tar.gz
RUN sudo tar -xf mujoco.tar.gz -C ${HOME}/.mujoco/
RUN sudo rm mujoco.tar.gz
RUN sudo python3 -m pip install seaborn hydra omegaconf pyquaternion debugpy
RUN sudo python3 -m pip install torch==1.13.1+cu117 torchvision==0.14.1+cu117 -f https://download.pytorch.org/whl/torch_stable.html

COPY requirements.txt ${HOME}/requirements.txt
RUN sudo python3 -m pip install -r /requirements.txt
COPY start.sh ${HOME}
WORKDIR ${HOME}
CMD ["/bin/bash", "start.sh"]
