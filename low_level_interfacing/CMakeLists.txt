cmake_minimum_required(VERSION 2.8.3)
project(low_level_interfacing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cl_tf
  geometry_msgs
  roslisp
  std_msgs
  tmc_msgs
  std_msgs
  move_base_msgs
  control_msgs
  #suturo_perception_msgs
  suturo_manipulation_msgs
  manipulation_action_msgs
  knowledge_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning_common
#  CATKIN_DEPENDS roslisp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_planning_common.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
