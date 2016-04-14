#pragma once

#include <string>

#include <ros/ros.h>

#include <message_filters/subscriber.h>

#include <tf/message_filter.h>
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

  /* Transform subs */

  ros::Subscriber subTransformCam0;
  ros::Subscriber subTransformCam1;
  ros::Subscriber subTransformCam2;
  ros::Subscriber subTransformCam3;

  /* Pose subs */
  message_filters::Subscriber<geometry_msgs::PoseStamped> subPoseCam0;
  message_filters::Subscriber<geometry_msgs::PoseStamped> subPoseCam1;
  message_filters::Subscriber<geometry_msgs::PoseStamped> subPoseCam2;
  message_filters::Subscriber<geometry_msgs::PoseStamped> subPoseCam3;

  tf::MessageFilter<geometry_msgs::PoseStamped> *tfFilterPoseCam0;
  tf::MessageFilter<geometry_msgs::PoseStamped> *tfFilterPoseCam1;
  tf::MessageFilter<geometry_msgs::PoseStamped> *tfFilterPoseCam2;
  tf::MessageFilter<geometry_msgs::PoseStamped> *tfFilterPoseCam3;  

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