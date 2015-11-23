cmake_minimum_required(VERSION 2.8.3)
project(ball_dropper_gui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ball_dropper_gui
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ball_dropper_sim src/ball_dropper_sim.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ball_dropper_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ball_dropper_sim
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
