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

add_executable(joint_position_test src/joint_position_test.cpp)
add_executable(pose_coords_test src/pose_coords_test.cpp)
add_executable(cartesian_coords_test src/cartesian_coords_test.cpp)

target_link_libraries(joint_position_test
   ${catkin_LIBRARIES}
 )

target_link_libraries(pose_coords_test
 ${catkin_LIBRARIES}
)

target_link_libraries(cartesian_coords_test
   ${catkin_LIBRARIES}
 )

