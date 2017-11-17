cmake_minimum_required(VERSION 2.8.3)
project(openarms)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

