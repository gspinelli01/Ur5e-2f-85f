FROM dustynv/ros:noetic-pytorch-l4t-r34.1.1

# System packages
RUN apt-get -y update
RUN apt-get -y install git ros-noetic-catkin cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential

# Install Universal robots repositories
RUN mkdir -p /home/catkin_ws
RUN cd /home/catkin_ws
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
RUN git clone git clone -b boost  https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
RUN git clone https://github.com/ros-industrial/universal_robot.git
# RUN git clone git@github.com:ciccio42/robotiq.git
# RUN git clone git@github.com:ciccio42/Ur5e-85f.git
# RUN git clone git@github.com:ciccio42/Realsense-Camera-Controller.git

# Black list for robotiq packages
RUN catkin config --skiplist robotiq_2f_140_gripper_visualization robotiq_2f_c2_gripper_visualization robotiq_3f_gripper_articulated_msgs robotiq_3f_gripper_articulated_gazebo robotiq_3f_gripper_articulated_gazebo_plugins robotiq_3f_gripper_control robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_visualization robotiq_3f_rviz robotiq_ft_sensor
RUN catkin make

