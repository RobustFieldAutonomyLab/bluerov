cmake_minimum_required(VERSION 2.8.3)
project(sonar_oculus)

add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
)

add_message_files(
   FILES
   OculusFire.msg
   OculusPing.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

generate_dynamic_reconfigure_options(
   cfg/OculusParams.cfg
 )

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_node src/sonar_oculus/sonar_oculus.cpp src/sonar_oculus/OculusClient.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${roslib_LIBRARIES}
)
