FROM ros_noetic:v1

SHELL ["/bin/bash", "-c"]
# System packages
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get -y install git cmake ros-noetic-catkin python3-catkin-tools python3-setuptools libgtest-dev build-essential
RUN apt-get update && apt-get install -y \
    wget \
    git \
    bash-completion \
    build-essential \
    sudo \ 
    && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/noetic/setup.bash

# Now create the same user as the host itself
# Add user
ARG USERNAME="gianl"
ARG USER_UID=1000
ARG USER_GID=1000
ARG GIT_PERSONAL_TOKEN=<YOUR_PERSONAL_GIT_TOKEN>

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
ENV PATH=/home/$USERNAME/.local/bin:$PATH

# Change HOME environment variable
ENV HOME /home/${USERNAME} 
# workspace setup
RUN mkdir -p ${HOME}/catkin_ws/src

# Install Universal robots repositories
RUN sudo apt-get update -y
RUN sudo apt-get install -y python3-wstool python3-rosdep
WORKDIR ${HOME}/catkin_ws/src
# Clone open-cv
RUN git clone -b noetic https://github.com/ros-perception/vision_opencv.git
WORKDIR ${HOME}/catkin_ws
RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin init
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build
RUN source devel/setup.bash

# Install moveit
WORKDIR ${HOME}/catkin_ws/src/moveit
RUN sudo apt-get purge -y '*opencv*' && sudo apt-get update -y && sudo wstool init . && sudo wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall && sudo wstool remove  moveit_tutorials &&  rosdep update && sudo wstool update -t . && rosdep install -y --from-paths . --ignore-src --rosdistro noetic --os=ubuntu:focal

# Clone and build UR-drivers and custom repositories
WORKDIR ${HOME}/catkin_ws
RUN git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git src/Universal_Robots_Client_Library
RUN sudo apt update -qq && sudo apt install python3-vcstool
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
RUN sudo vcs import --input src/Universal_Robots_ROS_Driver/.noetic.rosinstall src && rosdep update && rosdep install --from-paths src --ignore-src -y && sudo catkin build
RUN git config --global http.sslverify false
WORKDIR ${HOME}/catkin_ws/src
RUN git clone -b noetic_2f_85 https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/robotiq.git

# Repository dependencies
# Gripper dependencies
RUN rosdep install --from-paths . --ignore-src -y && sudo apt-get install ros-noetic-soem
WORKDIR ${HOME}/catkin_ws
RUN sudo catkin config --skiplist robotiq_2f_140_gripper_visualization robotiq_2f_c2_gripper_visualization robotiq_3f_gripper_articulated_msgs robotiq_3f_gripper_articulated_gazebo robotiq_3f_gripper_articulated_gazebo_plugins robotiq_3f_gripper_control robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_visualization robotiq_3f_rviz robotiq_ft_sensor
RUN sudo catkin build && source devel/setup.bash

# Joy dependencies
WORKDIR ${HOME}/catkin_ws/src
RUN mkdir ps4_joy_drivers
WORKDIR ${HOME}/catkin_ws/src/ps4_joy_drivers
RUN git clone https://github.com/chrippa/ds4drv.git
WORKDIR ${HOME}/catkin_ws/src/ps4_joy_drivers/ds4drv
RUN sudo apt-get install python3-pip udev -y && sudo python3 -m pip install --upgrade pip && sudo apt-get update && sudo apt-get install  python3-evdev && sudo python3 setup.py install && sudo mkdir -p /etc/udev/rules.d/ && sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/ && sudo /lib/systemd/systemd-udevd --daemon && sudo /lib/systemd/systemd-udevd --daemon && sudo udevadm control --reload-rules && sudo udevadm trigger
RUN git clone --branch noetic-devel  https://github.com/naoki-mizuno/ds4_driver.git
WORKDIR ${HOME}/catkin_ws
RUN sudo catkin build

# Install custom packages
WORKDIR ${HOME}/catkin_ws/src
RUN git clone https://$GIT_PERSONAL_TOKEN@github.com/gspinelli01/Ur5e-2f-85f.git
WORKDIR ${HOME}/catkin_ws
RUN sudo catkin build

# Install nvidia-driver
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin && \
    sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600  && \
    wget https://developer.download.nvidia.com/compute/cuda/11.7.0/local_installers/cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb && \
    sudo dpkg -i cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb && \
    sudo cp /var/cuda-repo-ubuntu2004-11-7-local/cuda-*-keyring.gpg /usr/share/keyrings/ && \
    sudo apt-get update
RUN sudo DEBIAN_FRONTEND=noninteractive apt-get -y install cuda-11-7
RUN sudo rm cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb
WORKDIR /home/gianl
# RUN echo "export PATH='/usr/local/cuda-11.7/bin:$PATH'" >> .bashrc && echo "export LD_LIBRARY_PATH='/usr/local/cuda-11.7/lib64:$LD_LIBRARY_PATH'" >> .bashrc

# Install Camera dependency
WORKDIR /home/temp
## ZED SDK 4.0.8
# RUN sudo wget -q --no-check-certificate https://download.stereolabs.com/zedsdk/4.0/cu117/ubuntu20
# RUN sudo chmod +x ubuntu20 && sudo apt install -y zstd && sudo ./ubuntu20 silent skip_tools && sudo rm -rf /usr/local/zed/resources/* && sudo rm -rf ubuntu20 && sudo rm -rf /var/lib/apt/lists/*
## ZED SDK 4.0.2
RUN sudo wget -q --no-check-certificate https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.0/ZED_SDK_Ubuntu20_cuda11.8_v4.0.2.zstd.run
RUN sudo chmod +x ZED_SDK_Ubuntu20_cuda11.8_v4.0.2.zstd.run && sudo apt install -y zstd && sudo ./ZED_SDK_Ubuntu20_cuda11.8_v4.0.2.zstd.run silent skip_tools && sudo rm -rf /usr/local/zed/resources/* && sudo rm -rf ZED_SDK_Ubuntu20_cuda11.8_v4.0.2.zstd.run && sudo rm -rf /var/lib/apt/lists/*
# # # This symbolic link is needed to use the streaming features on Jetson inside a container
RUN sudo ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so.0 && echo "export LD_LIBRARY_PATH='/usr/local/zed/lib:$LD_LIBRARY_PATH'" >> ${HOME}/.bashrc

# 4. Install ZED-sdk
WORKDIR ${HOME}/catkin_ws/src
RUN git clone https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/ZED-Controller.git
RUN git clone https://github.com/ciccio42/Learning-from-Demonstration-Dataset-Collector.git

WORKDIR ${HOME}
RUN git clone https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/Multi-Task-LFD-Framework.git && git clone https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/Multi-Task-LFD-Training-Framework.git  ${HOME}/Multi-Task-LFD-Framework/repo/Multi-Task-LFD-Training-Framework
RUN sudo python3 -m pip install pyquaternion && sudo python3 -m pip install ${HOME}/Multi-Task-LFD-Framework/repo/Multi-Task-LFD-Training-Framework/training/ 

WORKDIR ${HOME}/catkin_ws
RUN sudo catkin build  dataset_collector zed_camera_controller
RUN source devel/setup.bash
RUN sudo python3 -m pip install pynput

### in catkin_ws/src/Learning-from-Demonstration-Dataset-Collector/dataset_collector/src/trajectory_collector.py riga 13 cambiare `camera_controller` con `zed_camera_controller`

# ssh server
# RUN sudo apt-get -y update && sudo apt-get install -y openssh-server  && sudo mkdir /var/run/sshd && sudo echo 'root:root123' | sudo chpasswd && sudo sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# EXPOSE 22

CMD ["/bin/bash", "/home/docker_ubuntu20/start.sh"]