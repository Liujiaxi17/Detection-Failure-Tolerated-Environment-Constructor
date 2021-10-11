/** \file
 *
 *  ROS driver node for the grid map
 */
#include <ros/ros.h>
#include "rsgridmap.h"
#include "std_msgs/String.h"

using namespace rsgrid_map;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rsgridmap");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");



  // start the driver
  rsgrid_map::rsgridmap gmp(node, private_nh);
  ros::spin();
 
  return 0;
}