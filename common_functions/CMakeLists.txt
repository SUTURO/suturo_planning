cmake_minimum_required(VERSION 2.8.3)
project(common_functions)

find_package(catkin REQUIRED COMPONENTS
  roslisp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning_common
#  CATKIN_DEPENDS roslisp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/planning_common.cpp
# )

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
