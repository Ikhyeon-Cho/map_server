/*
 * map_server_node.cpp
 *
 *  Created on: Nov 23, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "occupancyMap_server/OccupancyMapServer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancyMap_server_node");
  ros::NodeHandle nh("~");

  ros::OccupancyMapServer node;

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}