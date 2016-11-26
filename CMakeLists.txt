cmake_minimum_required(VERSION 2.8.3)
project(kuka_io)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  genpy
  message_generation
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  A6.msg
  Status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
)
