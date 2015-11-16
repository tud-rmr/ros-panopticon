#include <string>

#include <ros/ros.h>
#include <ros/time.h>

#include "syncer_node.hpp"

using std::string;
using panopticon::SyncerNode;

int main(int argc, char **argv) {
  ros::init(argc, argv, "panopticon");

  ros::NodeHandle nh;

  string cameraOneNs = "/boreas/vrmagic";
  string cameraTwoNs = "/apollon/vrmagic";

  SyncerNode syncer(nh, cameraOneNs, cameraTwoNs);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
