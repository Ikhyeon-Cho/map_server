#include "occupancy_map/OccupancyGridMap.h"
#include <ros/console.h>

OccupancyGridMap::OccupancyGridMap() : GridMap({ "occupancy" })
{
}

OccupancyGridMap& OccupancyGridMap::operator=(const OccupancyGridMap& other)
{
  if (this != &other)
  {
    grid_map::GridMap::operator=(other);
  }
  return *this;
}

grid_map::GridMap::Matrix& OccupancyGridMap::getOccupancyLayer()
{
  return get("occupancy");
}

bool OccupancyGridMap::hasValidOccupancyLayer() const
{
  const auto& occupancy_layer = getOccupancyLayer();
  bool is_nan_layer = occupancy_layer.array().isNaN().all();

  if (is_nan_layer)
  {
    ROS_WARN_STREAM("Occupancy layer is empty. Occupancy msg conversion is failed.");
    return false;
  }
  return true;
}

const grid_map::GridMap::Matrix& OccupancyGridMap::getOccupancyLayer() const
{
  return get("occupancy");
}

std::tuple<float, float> OccupancyGridMap::getMinMaxOccupancyData() const
{
  const auto& data = getOccupancyLayer();

  // https://www.geeksforgeeks.org/difference-between-stdnumeric_limitst-min-max-and-lowest-in-cpp/
  auto fillNaNForFindingMaxVal = data.array().isNaN().select(std::numeric_limits<double>::lowest(), data);
  auto fillNaNForFindingMinVal = data.array().isNaN().select(std::numeric_limits<double>::max(), data);

  float min_value = fillNaNForFindingMinVal.minCoeff();
  float max_value = fillNaNForFindingMaxVal.maxCoeff();

  return { min_value, max_value };
}

bool OccupancyGridMap::isOutOfBoundaryAt(const grid_map::Index& grid_index)
{
  grid_map::Position pos;
  return !getPosition(grid_index, pos);
}

bool OccupancyGridMap::isOutOfBoundaryAt(const grid_map::Position& position)
{
  grid_map::Index idx;
  return !getIndex(position, idx);
}

bool OccupancyGridMap::isFreeAt(const grid_map::Index& grid_index) const
{
  const auto& state = at("occupancy", grid_index);
  return std::abs(state - FREE) < std::numeric_limits<float>::epsilon();
}

bool OccupancyGridMap::isOccupiedAt(const grid_map::Index& grid_index) const
{
  const auto& state = at("occupancy", grid_index);
  return std::abs(state - OCCUPIED) < std::numeric_limits<float>::epsilon();
}

bool OccupancyGridMap::isUnknownAt(const grid_map::Index& grid_index) const
{
  const auto& state = at("occupancy", grid_index);
  return std::abs(state - UNKNOWN) < std::numeric_limits<float>::epsilon();
}

grid_map::Position OccupancyGridMap::getPositionFrom(const grid_map::Index& grid_index) const
{
  grid_map::Position pos;
  getPosition(grid_index, pos);
  return pos;
}

grid_map::Index OccupancyGridMap::getIndexFrom(const grid_map::Position& grid_position) const
{
  grid_map::Index idx;
  getIndex(grid_position, idx);
  return idx;
}

float OccupancyGridMap::getDistance(const grid_map::Index& grid_index1, const grid_map::Index& grid_index2) const
{
  auto pos1 = getPositionFrom(grid_index1);
  auto pos2 = getPositionFrom(grid_index2);
  return (pos1 - pos2).norm();
}

grid_map::CircleIterator OccupancyGridMap::getCircleIterator(const grid_map::Index& query_index, double radius) const
{
  grid_map::Position queried_position;
  this->getPosition(query_index, queried_position);
  return grid_map::CircleIterator(*this, queried_position, radius);
}