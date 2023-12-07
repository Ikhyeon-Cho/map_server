cmake_minimum_required(VERSION 3.0.2)
project(elevation_map_loader)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_node_utils
  occupancy_grid_map
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp
    ros_node_utils
    occupancy_grid_map
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
)
target_link_libraries(elevation_map_loader_node
  ${catkin_LIBRARIES}
)