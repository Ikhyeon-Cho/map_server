/*
 * map_server_node.cpp
 *
 *  Created on: Nov 23, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "elevationmap_server/MapServer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevationmap_server_node");
  ros::NodeHandle nh("~");

  ros::MapServer node;

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}