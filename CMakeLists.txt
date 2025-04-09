cmake_minimum_required(VERSION 3.0.2)
project(rosneuro_pong)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  rosneuro_msgs
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Game.msg
  Paddle.msg
  Ball.msg
  GameInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
#   rosneuro_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES filters_tutorial
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
)


