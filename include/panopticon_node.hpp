#pragma once

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

namespace rmr {
class PanopticonNode {
 public:
  PanopticonNode(ros::NodeHandle _nh);
  ~PanopticonNode();

  void waitForWorldFrame(const std::string& topic);
  void spinOnce();

 private:
  ros::NodeHandle nh;

  ros::Subscriber subCam0;
  ros::Subscriber subCam1;
  ros::Subscriber subCam2;
  ros::Subscriber subCam3;

  tf::TransformBroadcaster br;

  geometry_msgs::TransformStamped origin;

  void cameraCallback(const geometry_msgs::TransformStamped::ConstPtr& msg, std::string suffix);

  void cameraCallback0(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void cameraCallback1(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void cameraCallback2(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void cameraCallback3(const geometry_msgs::TransformStamped::ConstPtr& msg);
};
}