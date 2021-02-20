cmake_minimum_required(VERSION 2.8.3)
project(handover_test)

find_package(catkin REQUIRED COMPONENTS
  kortex_driver
  roscpp
  rospy
  std_msgs
  message_generation
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS  std_msgs message_runtime
)
