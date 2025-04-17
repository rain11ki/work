cmake_minimum_required(VERSION 2.8.3)
project(dlrobot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rosserial_python
  serial
  sensor_msgs
  dl_msgs
  std_msgs
  tf
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dlrobot
  CATKIN_DEPENDS roscpp rospy tf nav_msgs dl_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dl_base src/dl_base.cpp)
add_executable(dl_base_node src/dl_base_node.cpp)
target_link_libraries(dl_base_node dl_base ${catkin_LIBRARIES})

