cmake_minimum_required(VERSION 2.8.3)
project(bluerov_launch)

find_package(catkin REQUIRED COMPONENTS
  bar30_depth
  bluerov_control
  rti_dvl
  sonar_oculus
  imu_vn_100
)

catkin_package()
