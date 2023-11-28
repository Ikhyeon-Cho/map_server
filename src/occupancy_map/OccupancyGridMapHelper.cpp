#include "occupancy_map/OccupancyGridMapHelper.h"

bool OccupancyGridMapHelper::initializeFromImage(const cv::Mat& image, const double resolution,
                                                 OccupancyGridMap& occupancy_map)
{
  grid_map::GridMapCvConverter::initializeFromImage(image, resolution, occupancy_map, grid_map::Position(0, 0));
  if (!grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(image, "image", occupancy_map, 0, 255))
    return false;

  const auto& image_layer = occupancy_map.get("image");
  auto& occupancy_layer = occupancy_map.getOccupancyLayer();

  occupancy_layer = (255.0f - image_layer.array()) / 255.0f;  // (255 -> FREE, 0 -> OCCUPIED)

  return true;
}

void OccupancyGridMapHelper::applyBinaryThreshold(double free_thres, double occupied_thres,
                                                  OccupancyGridMap& occupancy_map)
{
  auto& occupancy_layer = occupancy_map.getOccupancyLayer();

  // set free (occupancy: 0) over the threshold. Otherwise, Leave unchanged (nan)
  // Set Occupied (occupancy: 1) over the threshold.
  // Otherwise, set Free (occupancy: 0) below the threshold.
  // Remaining part is set to nan -> converted into -1 in the msg.
  occupancy_layer = occupancy_layer.unaryExpr([free_thres, occupied_thres](float value) {
    return (value >= occupied_thres) ? 1.0f : value <= free_thres ? 0.0f : NAN;
  });

  occupancy_layer =
      occupancy_layer.unaryExpr([occupied_thres](float value) { return (value > occupied_thres) ? 1.0f : value; });

  occupancy_layer =
      occupancy_layer.unaryExpr([free_thres](float value) { return (value < free_thres ? 0.0f : value); });

  occupancy_layer = occupancy_layer.unaryExpr([free_thres, occupied_thres](float value) {
    return (0 < value <= occupied_thres && value >= free_thres ? NAN : value);
  });
}

bool OccupancyGridMapHelper::getInflatedMap(const OccupancyGridMap& map, double inflation_radius,
                                            OccupancyGridMap& map_with_inflation)
{
  if (std::abs(inflation_radius) < DBL_EPSILON)
    return false;

  // Copy occupancy values to buffered occupancy layer
  const auto& occupancy_layer = map.getOccupancyLayer();
  map_with_inflation.getOccupancyLayer() = map.getOccupancyLayer();

  // save occupied indices in the map
  std::queue<grid_map::Index> occupied_indices;
  for (grid_map::GridMapIterator iter(map); !iter.isPastEnd(); ++iter)
  {
    const auto& grid_index = *iter;
    if (!map.isValid(grid_index))  // skip empty cell (NaN)
      continue;

    if (map.isFreeAt(grid_index))
      continue;

    occupied_indices.push(grid_index);
  }

  // make new occupied area to the map_with_inflation
  while (!occupied_indices.empty())
  {
    const auto& occupied_index = occupied_indices.front();
    const auto& occupied = occupancy_layer(occupied_index(0), occupied_index(1));

    grid_map::CircleIterator iter = map_with_inflation.getCircleIterator(occupied_index, inflation_radius);
    for (iter; !iter.isPastEnd(); ++iter)
    {
      const auto& index_to_be_occupied = *iter;

      if (map_with_inflation.isOutOfBoundaryAt(index_to_be_occupied))
        continue;

      auto& grid_state = map_with_inflation.at("occupancy", index_to_be_occupied);
      grid_state = occupied;

      // grid_map::Position occupied_position = map.getPositionFrom(occupied_index);
      // const auto &distanceFromObstacle = (occupied_position - position_to_be_occupied).norm();
      // auto inflationCost = 1 - distanceFromObstacle / _inflation_radius; // linear
      // auto inflationCost = 1 - 1 / (1 - std::exp(-_inflation_radius)) * (1 - std::exp(-distanceFromObstacle)); //
      // exponential if (inflationCost > inflationCellState) inflationCellState = inflationCost;
    }

    occupied_indices.pop();
  }

  return true;
}