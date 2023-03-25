cmake_minimum_required(VERSION 2.8.3)
project(pipeline)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ScanMatchingStatus.msg
)

add_service_files(FILES deep_srv.srv)

add_action_files(
  FILES
  MoveBase.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    message_runtime
    actionlib
    actionlib_msgs
    geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
