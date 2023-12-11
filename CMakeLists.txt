cmake_minimum_required(VERSION 3.0.2)
project(map_server)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_node_utils
  grid_map_core
  grid_map_ros
  grid_map_cv
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    occupancy_map
  CATKIN_DEPENDS
    roscpp
    ros_node_utils
    grid_map_core
    grid_map_ros
    grid_map_cv
    nav_msgs
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(occupancy_map
  src/occupancyMap/OccupancyGridMap.cpp
  src/occupancyMap/OccupancyGridMapHelper.cpp
  src/occupancyMap/OccupancyGridMapRosConverter.cpp
)
target_link_libraries(occupancy_map
  ${catkin_LIBRARIES}
)

################
## Executable ##
################

add_executable(occupancyMap_server_node src/occupancyMap_server_node.cpp
  src/occupancyMap_server/OccupancyMapServer.cpp
)
target_link_libraries(occupancyMap_server_node
  ${catkin_LIBRARIES}
  occupancy_map
)