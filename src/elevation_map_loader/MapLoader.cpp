/*
 * MapLoader.cpp
 *
 *  Created on: Nov 23, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "elevation_map_loader/MapLoader.h"

MapLoader::MapLoader()
{
  readOccupancyMapFromImage();

  visualization_timer_occupancy.start();
}

void MapLoader::readOccupancyMapFromImage()
{
  cv::Mat image = cv::imread(image_path.value());
  if (image.empty())
  {
    // should do something
  }

  if (!OccupancyGridMapHelper::initializeFromImage(image, grid_map_resolution.value(), occupancy_map_))
  {
    // should do something
  }

  OccupancyGridMapHelper::applyBinaryThreshold(occupancy_free_threshold.value(), occupancy_occupied_threshold.value(),
                                               occupancy_map_);
}

void MapLoader::visualizeOccupancyMap(const ros::TimerEvent& event)
{
  nav_msgs::OccupancyGrid msg;
  OccupancyGridMapRosConverter::toOccupancyGridMsg(occupancy_map_, msg);
  occupancy_map_publisher.publish(msg);
}