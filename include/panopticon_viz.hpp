#pragma once

#include <string>

#include <ros/ros.h>

#include <message_filters/subscriber.h>

#include <tf/message_filter.h>
#include <tf/transform_listener.h>

namespace rmr {
class PanopticonViz {
 public:
  PanopticonViz(ros::NodeHandle _nh);
  ~PanopticonViz();

 private:
  ros::NodeHandle nh;

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

  tf::TransformListener tfListener;

  /* Callbacks */
  void cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /* Helper methods */
};
}