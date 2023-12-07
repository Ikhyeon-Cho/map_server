/*
 * MapLoader.h
 *
 *  Created on: Nov 23, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef MAP_LOADER_H
#define MAP_LOADER_H

#include <ros/ros.h>
#include <ros_node_utils/Core.h>
#include <occupancy_grid_map/OccupancyGridMap.h>
#include <occupancy_grid_map/OccupancyGridMapHelper.h>
#include <occupancy_grid_map/OccupancyGridMapRosConverter.h>

class MapLoader
{
public:
  MapLoader();

  void readOccupancyMapFromImage();

  void visualizeOccupancyMap(const ros::TimerEvent& event);

public:
  // ROS Parameters : OccupancyMap
  roscpp::Parameter<std::string> image_path{ "OccupancyMap/PathToImage", "/home/isr" };
  roscpp::Parameter<double> grid_map_resolution{ "OccupancyMap/GridResolution", 0.1 };
  roscpp::Parameter<double> occupancy_free_threshold{ "OccupancyMap/OccupiedThreshold", 0.3 };
  roscpp::Parameter<double> occupancy_occupied_threshold{ "OccupancyMap/FreeThreshold", 0.7 };

  // ROS Things
  roscpp::Publisher<nav_msgs::OccupancyGrid> occupancy_map_publisher{ "topic" };
  roscpp::Timer visualization_timer_occupancy{ ros::Duration(2.0), &MapLoader::visualizeOccupancyMap, this };

private:
  OccupancyGridMap occupancy_map_;
};

#endif  // MAP_LOADER_H