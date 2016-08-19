#pragma once

#include <string>

#include <ros/ros.h>

#include <message_filters/subscriber.h>

#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

#include <boost/unordered_map.hpp> 

namespace rmr {
struct RawPosePublisher {
  ros::Publisher pubPoseCam0;
  ros::Publisher pubPoseCam1;
  ros::Publisher pubPoseCam2;
  ros::Publisher pubPoseCam3;

  ros::Publisher pubOdomCam0;
  ros::Publisher pubOdomCam1;
  ros::Publisher pubOdomCam2;
  ros::Publisher pubOdomCam3;

  RawPosePublisher() {}

  RawPosePublisher(ros::NodeHandle &nh, std::string markerName) {
    pubPoseCam0 = nh.advertise<geometry_msgs::PoseStamped>("pose/" + markerName + "/cam0", 1000);
    pubPoseCam1 = nh.advertise<geometry_msgs::PoseStamped>("pose/" + markerName + "/cam1", 1000);
    pubPoseCam2 = nh.advertise<geometry_msgs::PoseStamped>("pose/" + markerName + "/cam2", 1000);
    pubPoseCam3 = nh.advertise<geometry_msgs::PoseStamped>("pose/" + markerName + "/cam3", 1000);

    pubOdomCam0 = nh.advertise<nav_msgs::Odometry>("odom/" + markerName + "/cam0", 1000);
    pubOdomCam1 = nh.advertise<nav_msgs::Odometry>("odom/" + markerName + "/cam1", 1000);
    pubOdomCam2 = nh.advertise<nav_msgs::Odometry>("odom/" + markerName + "/cam2", 1000);
    pubOdomCam3 = nh.advertise<nav_msgs::Odometry>("odom/" + markerName + "/cam3", 1000);
  }
};

class PanopticonPoser {
 public:
  PanopticonPoser(ros::NodeHandle _nh);
  ~PanopticonPoser();

 private:
  ros::NodeHandle nh;

  /* Pose subs */
  message_filters::Subscriber<geometry_msgs::TransformStamped> subPoseCam0;
  message_filters::Subscriber<geometry_msgs::TransformStamped> subPoseCam1;
  message_filters::Subscriber<geometry_msgs::TransformStamped> subPoseCam2;
  message_filters::Subscriber<geometry_msgs::TransformStamped> subPoseCam3;

  tf::MessageFilter<geometry_msgs::TransformStamped> *tfFilterPoseCam0;
  tf::MessageFilter<geometry_msgs::TransformStamped> *tfFilterPoseCam1;
  tf::MessageFilter<geometry_msgs::TransformStamped> *tfFilterPoseCam2;
  tf::MessageFilter<geometry_msgs::TransformStamped> *tfFilterPoseCam3;  

  boost::unordered_map<std::string, RawPosePublisher> posePublishers;

  tf::TransformListener tfListener;

  /* Callbacks */
  void cameraPoseCallbackWithCamId(const geometry_msgs::TransformStamped::ConstPtr& msg, std::string camId);

  void cameraPoseCallback0(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void cameraPoseCallback1(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void cameraPoseCallback2(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void cameraPoseCallback3(const geometry_msgs::TransformStamped::ConstPtr& msg);

  /* Helper methods */
  geometry_msgs::PoseWithCovarianceStamped poseToPoseWithCovariance(const geometry_msgs::PoseStamped msg);
};
}
