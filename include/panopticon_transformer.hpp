#pragma once

#include <string>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>

namespace rmr {
class PanopticonTransformer {
 public:
  PanopticonTransformer(ros::NodeHandle _nh);
  ~PanopticonTransformer();

  void waitForWorldFrame(const std::string& topic);

 private:
  ros::NodeHandle nh;

  /* Transform subs */

  ros::Subscriber subTransformCam0;
  ros::Subscriber subTransformCam1;
  ros::Subscriber subTransformCam2;
  ros::Subscriber subTransformCam3;

  tf::TransformBroadcaster tfBroadcaster;

  geometry_msgs::TransformStamped origin;

  /* Callbacks */
  void cameraTransformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);

  /* Helper methods */
  void publishInversedTransformation(const geometry_msgs::TransformStamped::ConstPtr& msg);
};
}