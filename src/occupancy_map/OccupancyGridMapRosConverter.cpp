#include "occupancy_map/OccupancyGridMapRosConverter.h"

bool OccupancyGridMapRosConverter::toOccupancyGridMsg(const OccupancyGridMap& occupancy_map, nav_msgs::OccupancyGrid& msg)
{
  if (!occupancy_map.hasValidOccupancyLayer())
    return false;
  
  float min, max;
  std::tie(min, max) = occupancy_map.getMinMaxOccupancyData();
  grid_map::GridMapRosConverter::toOccupancyGrid(occupancy_map, "occupancy", min, max, msg);
  return true;
}