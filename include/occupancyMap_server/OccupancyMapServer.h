/*
 * OccupancyMapServer.h
 *
 *  Created on: Nov 23, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef MAP_SERVER_H
#define MAP_SERVER_H

#include <ros/ros.h>
#include <ros_node_utils/Core.h>
#include <occupancyMap/OccupancyGridMap.h>
#include <occupancyMap/OccupancyGridMapHelper.h>
#include <occupancyMap/OccupancyGridMapRosConverter.h>

namespace ros
{
class OccupancyMapServer
{
public:
  OccupancyMapServer();

  void readOccupancyMapFromImage();

  void visualizeOccupancyMap(const ros::TimerEvent& event);

public:
  // Published Topics
  roscpp::Parameter<std::string> map_topic{ "~/Published_Topics/map", "/map" };

  // Parameters
  // -- Occupancy Map
  roscpp::Parameter<std::string> image_path{ "~/Parameters/image_path", "/home/isr" };
  roscpp::Parameter<double> grid_resolution{ "~/Parameters/grid_resolution", 0.1 };
  roscpp::Parameter<double> occupancy_free_threshold{ "~/Parameters/occupancy_free_threshold", 0.3 };
  roscpp::Parameter<double> occupancy_occupied_threshold{ "~/Parameters/occupancy_occupied_threshold", 0.7 };

  // -- Duration
  roscpp::Parameter<double> map_visualization_duration{ "~/Parameters/map_visualization_duration", 2.0 };

private:
  OccupancyGridMap occupancy_map_;

  roscpp::Publisher<nav_msgs::OccupancyGrid> occupancy_map_publisher{ map_topic.param() };
  roscpp::Timer visualization_timer_occupancy{ map_visualization_duration.param(),
                                               &OccupancyMapServer::visualizeOccupancyMap, this };
};

}  // namespace ros

#endif  // MAP_SERVER_H