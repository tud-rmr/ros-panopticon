#include "panopticon_poser.hpp"

#include <ros/topic.h>

#include <boost/algorithm/string/predicate.hpp>

using std::string;

namespace rmr {

PanopticonPoser::PanopticonPoser(ros::NodeHandle _nh) {
  nh = _nh;

  subPoseCam0.subscribe(nh, "camera0/pose", 100);
  subPoseCam1.subscribe(nh, "camera1/pose", 100);
  subPoseCam2.subscribe(nh, "camera2/pose", 100);
  subPoseCam3.subscribe(nh, "camera3/pose", 100);

  tfFilterPoseCam0 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam0, tfListener, "usb_cam0", 100);
  tfFilterPoseCam1 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam1, tfListener, "usb_cam1", 100);
  tfFilterPoseCam2 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam2, tfListener, "usb_cam2", 100);
  tfFilterPoseCam3 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam3, tfListener, "usb_cam3", 100);

  tfFilterPoseCam0->registerCallback(&PanopticonPoser::cameraPoseCallback, this);
  tfFilterPoseCam1->registerCallback(&PanopticonPoser::cameraPoseCallback, this);
  tfFilterPoseCam2->registerCallback(&PanopticonPoser::cameraPoseCallback, this);
  tfFilterPoseCam3->registerCallback(&PanopticonPoser::cameraPoseCallback, this);

  pubPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1000);
}

PanopticonPoser::~PanopticonPoser() {
  delete tfFilterPoseCam0;
  delete tfFilterPoseCam1;
  delete tfFilterPoseCam2;
  delete tfFilterPoseCam3;
}

void PanopticonPoser::cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  string frameName = msg->header.frame_id;

  if (frameName != "map" && frameName != "center") {
    try{
      geometry_msgs::PoseStamped markerInWorldMsg;
      tfListener.waitForTransform("map", frameName, ros::Time(0), ros::Duration(3.0));
      tfListener.transformPose("map", ros::Time::now(), *msg, "map", markerInWorldMsg);
      geometry_msgs::PoseWithCovarianceStamped poseWithCovMsg = poseToPoseWithCovariance(*msg);
      pubPose.publish(poseWithCovMsg);    
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
  }
}

geometry_msgs::PoseWithCovarianceStamped 
PanopticonPoser::poseToPoseWithCovariance(const geometry_msgs::PoseStamped poseMsg) {
  geometry_msgs::PoseWithCovarianceStamped poseWithCovMsg;

  poseWithCovMsg.header = poseMsg.header;
  poseWithCovMsg.pose.pose = poseMsg.pose;

    // TODO: do something intelligent about the covariance?
  float covariance[] = {
    1e-3, 0, 0, 0, 0, 0, 
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e-3, 0, 0, 0,
    0, 0, 0, 1e-3, 0, 0,
    0, 0, 0, 0, 1e-3, 0,
    0, 0, 0, 0, 0, 1e-3
  };

  for (unsigned int i = 0; i < poseWithCovMsg.pose.covariance.size(); i++) {
    poseWithCovMsg.pose.covariance[i] = covariance[i];
  }

  return poseWithCovMsg;
}

}