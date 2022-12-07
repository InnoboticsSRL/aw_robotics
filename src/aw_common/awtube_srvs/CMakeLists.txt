cmake_minimum_required(VERSION 2.8.3)

project(aware_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation  # Needed to generate custom message types
  geometry_msgs
  std_msgs
  aware_msgs
)

add_service_files(  # Tell catkin which custom message files to include
  FILES
  RobotStateService.srv
  Integer.srv
  GetBool.srv
  GetUint16.srv
  SetUint8.srv
  ReadInputBeckhoff.srv
  SetOutputBeckhoff.srv
  ReadOutputBeckhoff.srv
  GetFdbkGripper.srv
  SetCmdGripper.srv
  SafetyAreaMng.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  aware_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime # Needed to generate custom messages
)
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
)
