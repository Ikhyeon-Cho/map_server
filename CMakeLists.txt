cmake_minimum_required(VERSION 3.0.2)
project(elevation_map_loader)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
  nav_msgs
  tf
  tf2_ros
  ddynamic_reconfigure
  isr_ros_utils
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp
    grid_map_core
    grid_map_ros
    grid_map_msgs
    nav_msgs
    tf
    tf2_ros
    ddynamic_reconfigure
    isr_ros_utils
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################
## Executable ##
################

add_executable(elevation_map_loader_node src/map_loader_node.cpp
  src/elevation_map_loader/MapLoader.cpp
  src/occupancy_map/OccupancyGridMap.cpp
  src/occupancy_map/OccupancyGridMapHelper.cpp
  src/occupancy_map/OccupancyGridMapRosConverter.cpp
)
target_link_libraries(elevation_map_loader_node
  ${catkin_LIBRARIES}
)