FROM dustynv/ros:noetic-pytorch-l4t-r34.1.1

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
ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG GIT_PERSONAL_TOKEN

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install open-cv
COPY install_open_cv.sh install_open_cv.sh
RUN source install_open_cv.sh

# Set the default user. Omit if you want to keep the default as root.
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
RUN sudo wstool init .
RUN sudo wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
RUN sudo wstool remove  moveit_tutorials  # this is cloned in the next section
RUN rosdep update
RUN sudo wstool update -t . && rosdep install -y --from-paths . --ignore-src --rosdistro noetic --os=ubuntu:focal

# Clone and build UR-drivers and custom repositories
WORKDIR ${HOME}/catkin_ws/src
RUN sudo apt-get remove -y ros-noetic-ros-control
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
RUN git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
RUN git clone https://github.com/ros-industrial/universal_robot.git
RUN rosdep install -y --from-paths . --ignore-src --rosdistro noetic --os=ubuntu:focal

WORKDIR ${HOME}/catkin_ws
RUN sudo catkin build
RUN source devel/setup.bash

RUN git config --global http.sslverify false
WORKDIR ${HOME}/catkin_ws/src
RUN git clone -b noetic_2f_85 https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/robotiq.git

# Repository dependencies
# 1. Gripper dependencies
RUN rosdep install --from-paths . --ignore-src -y
RUN sudo apt-get install ros-noetic-soem
WORKDIR ${HOME}/catkin_ws
RUN catkin config --skiplist robotiq_2f_140_gripper_visualization robotiq_2f_c2_gripper_visualization robotiq_3f_gripper_articulated_msgs robotiq_3f_gripper_articulated_gazebo robotiq_3f_gripper_articulated_gazebo_plugins robotiq_3f_gripper_control robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_visualization robotiq_3f_rviz robotiq_ft_sensor
RUN sudo catkin build
RUN source devel/setup.bash

# 2. Joy dependencies
WORKDIR ${HOME}/catkin_ws/src
RUN mkdir ps4_joy_drivers
WORKDIR ${HOME}/catkin_ws/src/ps4_joy_drivers
RUN git clone https://github.com/chrippa/ds4drv.git
WORKDIR ${HOME}/catkin_ws/src/ps4_joy_drivers/ds4drv
RUN python3 -m pip install --upgrade pip
RUN sudo python3 setup.py install
RUN sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
RUN sudo /lib/systemd/systemd-udevd --daemon
RUN sudo /lib/systemd/systemd-udevd --daemon && sudo udevadm control --reload-rules && sudo udevadm trigger
RUN git clone --branch noetic-devel  https://github.com/naoki-mizuno/ds4_driver.git
WORKDIR ${HOME}/catkin_ws
RUN sudo catkin build

# 3. Install Camera dependency
WORKDIR /home/temp
RUN sudo wget -q --no-check-certificate https://download.stereolabs.com/zedsdk/4.0/l4t35.2/jetsons
RUN sudo chmod +x jetsons
RUN sudo apt install -y zstd
RUN sudo ./jetsons silent skip_tools && sudo rm -rf /usr/local/zed/resources/* && sudo rm -rf jetsons && sudo rm -rf /var/lib/apt/lists/*
# This symbolic link is needed to use the streaming features on Jetson inside a container
RUN sudo ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# # Install custom packages
WORKDIR ${HOME}/catkin_ws/src
RUN git clone https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/Ur5e-2f-85f.git
WORKDIR ${HOME}/catkin_ws
RUN sudo catkin build

# 4. Install ZED-sdk
WORKDIR ${HOME}/catkin_ws/src
RUN git clone https://$GIT_PERSONAL_TOKEN@github.com/ciccio42/ZED-Controller.git
# RUN sudo catkin build zed_camera_controller

CMD ["/bin/bash", "/home/mivia/docker_jetson_xavier_nx/start.sh"]