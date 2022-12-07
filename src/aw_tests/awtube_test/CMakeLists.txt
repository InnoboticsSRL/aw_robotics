cmake_minimum_required(VERSION 3.0.2)
project(awtube_test)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES awtube_test
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(awtube_joint_position_test src/awtube_joint_position_test.cpp)
add_executable(awtube_pose_test src/awtube_pose_test.cpp)

target_link_libraries(awtube_joint_position_test
   ${catkin_LIBRARIES}
 )

target_link_libraries(awtube_pose_test
 ${catkin_LIBRARIES}
)

