cmake_minimum_required(VERSION 2.8.3)
project(go_get_it_demo)

find_package(catkin REQUIRED COMPONENTS
  roslisp
  std_msgs
  nlp_msgs
  cl_tf2
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

add_lisp_executable(
execute-go-get-it go-get-it-demo clean::execute-go-get-it
)
