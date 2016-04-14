#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/time.h>

#include "panopticon_transformer_node.hpp"
#include "panopticon_poser_node.hpp"

using std::cin;
using std::string;

using namespace rmr;

int main(int argc, char **argv) {
  ros::init(argc, argv, "panopticon");

  ROS_INFO("Starting panopticon node");
  ros::NodeHandle nh("panopticon");

  PanopticonTransformer panopticonTransformer(nh);
  PanopticonPoser panopticonPoser(nh);

  /* Wait for the user to lay down the world marker*/
  ROS_INFO("%s%sPut the world marker on the floor and press [ENTER] to continue!", "\33[", "32m");
  // cin.clear();
  // cin.ignore();

  // ROS_INFO("Waiting for world frame");
  // panopticon.waitForWorldFrame("/panopticon/camera4/transform");
  // ROS_INFO("Found world frame!");

  /* Event loop*/

  ros::Rate r(100);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
