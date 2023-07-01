cmake_minimum_required(VERSION 3.1.3)
project(ur5e_2f_85_camera_table_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  robotiq_2f_85_gripper_visualization
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
