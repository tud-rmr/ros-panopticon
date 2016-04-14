#include "panopticon_node.hpp"

#include <ros/topic.h>

#include <boost/algorithm/string/predicate.hpp>

using std::string;

namespace rmr {

PanopticonNode::PanopticonNode(ros::NodeHandle _nh) {
  nh = _nh;

  subCam0 = nh.subscribe("camera0/transform", 1000, &PanopticonNode::cameraCallback0, this);
  subCam1 = nh.subscribe("camera1/transform", 1000, &PanopticonNode::cameraCallback1, this);
  subCam2 = nh.subscribe("camera2/transform", 1000, &PanopticonNode::cameraCallback2, this);
  subCam3 = nh.subscribe("camera3/transform", 1000, &PanopticonNode::cameraCallback3, this);
}

PanopticonNode::~PanopticonNode() {}

void PanopticonNode::waitForWorldFrame(const string& topic) {
  boost::shared_ptr<geometry_msgs::TransformStamped const> msgPtr;

  do {
    msgPtr = ros::topic::waitForMessage<geometry_msgs::TransformStamped>(topic, nh);
    if (NULL == msgPtr) {
      ROS_ERROR("waitForMessage was interrupted by node shutting down!");
      continue;
    }

  } while (msgPtr->child_frame_id != "raw_origin");

  origin = *msgPtr;
}

/*
void PanopticonNode::cameraCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  tf::StampedTransform st;
  if (msg->child_frame_id == "map" || msg->header.frame_id == "odom" ) {    
    tf::transformStampedMsgToTF(*msg, st);    
  } else {
    tf::Transform tr;
    tf::transformMsgToTF(msg->transform, tr);
    st = tf::StampedTransform(tr.inverse(), ros::Time::now(), msg->child_frame_id, msg->header.frame_id);    
  }
  br.sendTransform(st);
}
*/

void PanopticonNode::cameraCallback(const geometry_msgs::TransformStamped::ConstPtr& msg, string suffix) {
  string boardName = msg->child_frame_id;

  if(boardName != "map" && boardName != "odom")
     boardName += "@" + suffix;

  tf::Transform tr;
  tf::transformMsgToTF(msg->transform, tr);
  tf::StampedTransform inversed(tr, ros::Time::now(), msg->header.frame_id, boardName); 
  br.sendTransform(inversed);
}

void PanopticonNode::cameraCallback0(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  cameraCallback(msg, "0");
}

void PanopticonNode::cameraCallback1(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  cameraCallback(msg, "1");
}

void PanopticonNode::cameraCallback2(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  cameraCallback(msg, "2");
}

void PanopticonNode::cameraCallback3(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  cameraCallback(msg, "3");
}

void PanopticonNode::spinOnce() {}
}