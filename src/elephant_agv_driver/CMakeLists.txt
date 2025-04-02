cmake_minimum_required(VERSION 3.0.2)
project(elephant_agv_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  message_generation
  rosparam
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    geometry_msgs 
    std_msgs 
    message_runtime
    rosparam
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/elephant_agv_node.py
  scripts/test_mecanum.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
) 