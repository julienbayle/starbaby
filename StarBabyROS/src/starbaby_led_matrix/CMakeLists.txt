cmake_minimum_required(VERSION 2.8.3)
project(starbaby_led_matrix)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Eye.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
