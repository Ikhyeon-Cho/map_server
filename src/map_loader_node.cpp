/*
 * map_loader_node.cpp
 *
 *  Created on: Nov 23, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "elevation_map_loader/MapLoader.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_map_loader_node");
  ros::NodeHandle nh("~");

  MapLoader node;

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}