cmake_minimum_required(VERSION 3.0.2)
project(swarm_robots_formation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
#  LIBRARIES 
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


add_executable( relative_distance_calculator src/relative_distance_calculator )
add_executable( listener src/listener )
target_link_libraries(relative_distance_calculator  ${catkin_LIBRARIES} )
target_link_libraries(listener  ${catkin_LIBRARIES} )
