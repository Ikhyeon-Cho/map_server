cmake_minimum_required(VERSION 3.0.2)
project(elevationmap_server)
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

add_executable(elevationmap_server_node src/map_server_node.cpp
  src/elevationmap_server/MapServer.cpp
)
target_link_libraries(elevationmap_server_node
  ${catkin_LIBRARIES}
)