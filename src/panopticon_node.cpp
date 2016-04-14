#include "panopticon_node.hpp"

#include <ros/topic.h>

using std::string;

namespace rmr {

PanopticonNode::PanopticonNode(ros::NodeHandle _nh) {
  nh = _nh;

  subTransformCam0 = nh.subscribe("camera0/transform", 1000, &PanopticonNode::cameraTransformCallback, this);
  subTransformCam1 = nh.subscribe("camera1/transform", 1000, &PanopticonNode::cameraTransformCallback, this);
  subTransformCam2 = nh.subscribe("camera2/transform", 1000, &PanopticonNode::cameraTransformCallback, this);
  subTransformCam3 = nh.subscribe("camera3/transform", 1000, &PanopticonNode::cameraTransformCallback, this);

  subPoseCam0 = nh.subscribe("camera0/pose", 1000, &PanopticonNode::cameraPoseCallback, this);
  subPoseCam1 = nh.subscribe("camera1/pose", 1000, &PanopticonNode::cameraPoseCallback, this);
  subPoseCam2 = nh.subscribe("camera2/pose", 1000, &PanopticonNode::cameraPoseCallback, this);
  subPoseCam3 = nh.subscribe("camera3/pose", 1000, &PanopticonNode::cameraPoseCallback, this);

  pubPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1000);
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

  } while (msgPtr->child_frame_id != "map");

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

void PanopticonNode::cameraTransformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  string frameName = msg->child_frame_id;
  string parentName = msg->header.frame_id;

  if(frameName == "map" && parentName == "usb_cam0") {
    publishInversedTransformation(msg);
  } else if(frameName == "center") {
    if(parentName == "usb_cam0") {
      tf::StampedTransform st;
      tf::transformStampedMsgToTF(*msg, st);
      tfBroadcaster.sendTransform(st);  
    } else {
      publishInversedTransformation(msg);
    }
  }
}

void PanopticonNode::cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  string frameName = msg->header.frame_id;
  /*
  if (frameName != "map" && frameName != "raw_center") {
    geometry_msgs::PoseStamped markerInWorldMsg;
    tfListener.waitForTransform("map", frameName, ros::Time::now(), ros::Duration(3.0));
    tfListener.transformPose("map", *msg, markerInWorldMsg);
    geometry_msgs::PoseWithCovarianceStamped poseWithCovMsg = poseToPoseWithCovariance(*msg);
    pubPose.publish(poseWithCovMsg);
  }
  */
}

geometry_msgs::PoseWithCovarianceStamped 
PanopticonNode::poseToPoseWithCovariance(const geometry_msgs::PoseStamped poseMsg) {
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

void PanopticonNode::publishInversedTransformation(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  string frameName = msg->child_frame_id;
  string parentName = msg->header.frame_id;
  tf::Transform tr;
  tf::transformMsgToTF(msg->transform, tr);
  tf::StampedTransform inversed(tr.inverse(), ros::Time::now(), frameName, parentName);
  tfBroadcaster.sendTransform(inversed);     
}

}