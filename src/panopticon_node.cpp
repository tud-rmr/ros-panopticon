#include "panopticon_node.hpp"

#include <ros/topic.h>

#include <boost/algorithm/string/predicate.hpp>

using std::string;

namespace rmr {

PanopticonNode::PanopticonNode(ros::NodeHandle _nh) {
  nh = _nh;

  subCam4 = nh.subscribe("camera4/transform", 1000, &PanopticonNode::cameraCallback, this);
  subCam5 = nh.subscribe("camera5/transform", 1000, &PanopticonNode::cameraCallback, this);
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

void PanopticonNode::cameraCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  if (boost::algorithm::starts_with(msg->child_frame_id, "board")) {
    tf::Transform tr;
    tf::transformMsgToTF(msg->transform, tr);

    tf::StampedTransform inversed(tr.inverse(), ros::Time::now(), "mid_marker", msg->header.frame_id);

    br.sendTransform(inversed);
  }
}

void PanopticonNode::spinOnce() {}
}