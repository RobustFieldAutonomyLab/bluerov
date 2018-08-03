cmake_minimum_required(VERSION 2.8)
project(bluerov_control)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy

  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  # planner plugin
  pluginlib
  nav_core
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(fake_map src/fake_map.cpp)
target_link_libraries(fake_map ${catkin_LIBRARIES})
