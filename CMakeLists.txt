cmake_minimum_required(VERSION 3.0.2)
project(skills_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  genmsg
  actionlib_msgs
  actionlib
  actionlib_tools
  message_generation
  platonics_vision
  rospkg
  dynamic_reconfigure
  cv_bridge
  panda_ros
  tf
)


catkin_python_setup()

add_service_files(
  FILES
  ListTrajectories.srv
  ListTemplates.srv
  ListSkills.srv
)

add_action_files(
  DIRECTORY action
  FILES
  LfdHome.action
  LfdRecord.action
  LfdExecuteSequence.action
  LfdExecuteSkill.action
  AbortGripper.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package()



