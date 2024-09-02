cmake_minimum_required(VERSION 3.0.2)
project(skills_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  actionlib_msgs
  actionlib
  message_generation
)


catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES LfdHome.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package()



