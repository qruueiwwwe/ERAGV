cmake_minimum_required(VERSION 3.0.2)
project(elephant_agv_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  message_generation
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/elephant_agv_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 