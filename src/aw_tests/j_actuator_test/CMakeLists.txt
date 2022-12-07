cmake_minimum_required(VERSION 3.0.2)
project(j_actuator_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
)

catkin_package(
CATKIN_DEPENDS 
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(j_actuator_test src/j_actuator_test.cpp)

target_link_libraries(j_actuator_test
   ${catkin_LIBRARIES}
 )

