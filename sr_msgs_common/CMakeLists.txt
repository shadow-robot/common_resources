cmake_minimum_required(VERSION 2.8.3)
project(sr_msgs_common)

set(MSG_DEPS
  std_msgs
  shape_msgs
  geometry_msgs
  actionlib_msgs
  moveit_msgs
)

find_package(catkin REQUIRED COMPONENTS roslib genmsg message_generation ${MSG_DEPS})

add_message_files(
  FILES
  GraspObject.msg
  AvailableObjects.msg
  FingersInContact.msg
  SqueezeDirection.msg
  Grasp.msg
  GraspCommand.msg
)

add_service_files(
  FILES
  GetGraspForKnownObject.srv
  GetPoseByName.srv
)

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES
   Grasp.action
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roslib message_runtime ${MSG_DEPS}
  INCLUDE_DIRS
  LIBRARIES
)
