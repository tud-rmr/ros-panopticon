#include "panopticon_node.hpp"

#include <ros/topic.h>

#include <boost/algorithm/string/predicate.hpp>

using std::string;

namespace rmr {

PanopticonNode::PanopticonNode(ros::NodeHandle _nh) {
  nh = _nh;

  subTransformCam0 = nh.subscribe("camera0/transform", 1000, &PanopticonNode::cameraTransformCallback, this);
  subTransformCam1 = nh.subscribe("camera1/transform", 1000, &PanopticonNode::cameraTransformCallback, this);
  subTransformCam2 = nh.subscribe("camera2/transform", 1000, &PanopticonNode::cameraTransformCallback, this);
  subTransformCam3 = nh.subscribe("camera3/transform", 1000, &PanopticonNode::cameraTransformCallback, this);

  subPoseCam0.subscribe(nh, "camera0/pose", 100);
  subPoseCam1.subscribe(nh, "camera1/pose", 100);
  subPoseCam2.subscribe(nh, "camera2/pose", 100);
  subPoseCam3.subscribe(nh, "camera3/pose", 100);

  tfFilterPoseCam0 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam0, tfListener, "usb_cam0", 100);
  tfFilterPoseCam1 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam1, tfListener, "usb_cam1", 100);
  tfFilterPoseCam2 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam2, tfListener, "usb_cam2", 100);
  tfFilterPoseCam3 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam3, tfListener, "usb_cam3", 100);

  tfFilterPoseCam0->registerCallback(&PanopticonNode::cameraPoseCallback, this);
  tfFilterPoseCam1->registerCallback(&PanopticonNode::cameraPoseCallback, this);
  tfFilterPoseCam2->registerCallback(&PanopticonNode::cameraPoseCallback, this);
  tfFilterPoseCam3->registerCallback(&PanopticonNode::cameraPoseCallback, this);

  pubPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1000);
}

PanopticonNode::~PanopticonNode() {
  delete tfFilterPoseCam0;
  delete tfFilterPoseCam1;
  delete tfFilterPoseCam2;
  delete tfFilterPoseCam3;
}

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
      st.stamp_ = ros::Time::now();
      tf::transformStampedMsgToTF(*msg, st);
      tfBroadcaster.sendTransform(st);  
    } else {
      publishInversedTransformation(msg);
    }
  }
}

void PanopticonNode::cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  string frameName = msg->header.frame_id;

  if (frameName != "map" && frameName != "center") {
    try{
      geometry_msgs::PoseStamped markerInWorldMsg;
      tfListener.transformPose("map", ros::Time::now(), *msg, "map", markerInWorldMsg);
      geometry_msgs::PoseWithCovarianceStamped poseWithCovMsg = poseToPoseWithCovariance(*msg);
      pubPose.publish(poseWithCovMsg);    
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
  }
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