cmake_minimum_required(VERSION 2.8.3)
project(ball_dropper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    ball_dropper_msgs
    message_generation
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   ball_dropper_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ball_dropper
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(ball_dropper_node src/ball_dropper.cpp src/rdt.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ball_dropper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ball_dropper_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

