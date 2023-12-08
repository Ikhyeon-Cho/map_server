/*
 * MapServer.cpp
 *
 *  Created on: Nov 23, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "elevationmap_server/MapServer.h"

namespace ros
{
MapServer::MapServer()
{
  readOccupancyMapFromImage();

  visualization_timer_occupancy.start();
}

void MapServer::readOccupancyMapFromImage()
{
  cv::Mat image = cv::imread(image_path.param());
  if (image.empty())
  {
    std::cout << "Reading map failed \n";
    return;
  }

  if (!OccupancyGridMapHelper::initializeFromImage(image, grid_map_resolution.param(), occupancy_map_))
  {
    std::cout << "converting from image to map failed \n";
    return;
  }

  OccupancyGridMapHelper::applyBinaryThreshold(occupancy_free_threshold.param(), occupancy_occupied_threshold.param(),
                                               occupancy_map_);
}

void MapServer::visualizeOccupancyMap(const ros::TimerEvent& event)
{
  nav_msgs::OccupancyGrid msg;
  OccupancyGridMapRosConverter::toOccupancyGridMsg(occupancy_map_, msg);
  occupancy_map_publisher.publish(msg);
}

}  // namespace ros
