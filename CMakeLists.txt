cmake_minimum_required(VERSION 2.8.3)
project(mission_manager)

find_package(catkin REQUIRED COMPONENTS
  mission_plan
  rospy
  std_msgs
  geometry_msgs
  project11_transformations
  tf2
  message_generation
  dynamic_reconfigure
)

catkin_python_setup()

 add_message_files(
   FILES
   BehaviorControl.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

generate_dynamic_reconfigure_options(
    cfg/mission_manager.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES missionmanager
  CATKIN_DEPENDS mission_plan rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

