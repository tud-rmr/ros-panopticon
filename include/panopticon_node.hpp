#pragma once

#include <string>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace rmr {
class PanopticonNode {
 public:
  PanopticonNode(ros::NodeHandle _nh);
  ~PanopticonNode();

  void waitForWorldFrame(const std::string& topic);
  void spinOnce();

 private:
  ros::NodeHandle nh;

  ros::Subscriber subTransformCam0;
  ros::Subscriber subTransformCam1;
  ros::Subscriber subTransformCam2;
  ros::Subscriber subTransformCam3;

  ros::Subscriber subPoseCam0;
  ros::Subscriber subPoseCam1;
  ros::Subscriber subPoseCam2;
  ros::Subscriber subPoseCam3;

  ros::Publisher pubPose;

  tf::TransformBroadcaster tfBroadcaster;
  tf::TransformListener tfListener;

  geometry_msgs::TransformStamped origin;

  /* Callbacks */
  void cameraTransformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /* Helper methods */
  geometry_msgs::PoseWithCovarianceStamped poseToPoseWithCovariance(const geometry_msgs::PoseStamped msg);
  void publishInversedTransformation(const geometry_msgs::TransformStamped::ConstPtr& msg);
};
}