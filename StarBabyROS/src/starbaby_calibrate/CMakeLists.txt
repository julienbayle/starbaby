cmake_minimum_required(VERSION 2.8.3)
project(starbaby_calibrate)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
  message_generation
)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES Calibrate.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs
)

