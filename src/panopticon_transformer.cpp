#include "panopticon_transformer.hpp"

#include <ros/topic.h>

#include <boost/algorithm/string/predicate.hpp>

using std::string;

namespace rmr {

PanopticonTransformer::PanopticonTransformer(ros::NodeHandle _nh) {
  nh = _nh;

  frameContainingMap = "usb_cam0";

  subTransformCam0 = nh.subscribe("camera0/transform", 1000, &PanopticonTransformer::cameraTransformCallback, this);
  subTransformCam1 = nh.subscribe("camera1/transform", 1000, &PanopticonTransformer::cameraTransformCallback, this);
  subTransformCam2 = nh.subscribe("camera2/transform", 1000, &PanopticonTransformer::cameraTransformCallback, this);
  subTransformCam3 = nh.subscribe("camera3/transform", 1000, &PanopticonTransformer::cameraTransformCallback, this);
}

PanopticonTransformer::~PanopticonTransformer() {}

void PanopticonTransformer::waitForWorldFrame(const string& topic) {
  boost::shared_ptr<geometry_msgs::TransformStamped const> msgPtr;

  do {
    msgPtr = ros::topic::waitForMessage<geometry_msgs::TransformStamped>(topic, nh);
    if (NULL == msgPtr) {
      ROS_ERROR("waitForMessage was interrupted by node shutting down!");
      continue;
    }

  } while (msgPtr->child_frame_id != "map");

  origin = *msgPtr;
}

void PanopticonTransformer::cameraTransformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  string frameName = msg->child_frame_id;
  string parentName = msg->header.frame_id;

  if(frameName == "map" && parentName == frameContainingMap) {
    publishInversedTransformation(msg);
  } else if(frameName == "center") {
    if(parentName == frameContainingMap) {
      tf::StampedTransform st;
      st.stamp_ = ros::Time::now();
      tf::transformStampedMsgToTF(*msg, st);
      tfBroadcaster.sendTransform(st);  
    } else {
      publishInversedTransformation(msg);
    }
  }
}

void PanopticonTransformer::publishInversedTransformation(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  string frameName = msg->child_frame_id;
  string parentName = msg->header.frame_id;
  tf::Transform tr;
  tf::transformMsgToTF(msg->transform, tr);
  tf::StampedTransform inversed(tr.inverse(), ros::Time::now(), frameName, parentName);
  tfBroadcaster.sendTransform(inversed);     
}

}