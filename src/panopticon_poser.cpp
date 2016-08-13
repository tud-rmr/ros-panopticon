#include "panopticon_poser.hpp"

#include <ros/topic.h>

#include <boost/algorithm/string/predicate.hpp>

using std::string;

namespace rmr {

PanopticonPoser::PanopticonPoser(ros::NodeHandle _nh) {
  nh = _nh;

  subPoseCam0.subscribe(nh, "camera0/transform", 10);
  subPoseCam1.subscribe(nh, "camera1/transform", 10);
  subPoseCam2.subscribe(nh, "camera2/transform", 10);
  subPoseCam3.subscribe(nh, "camera3/transform", 10);

  tfFilterPoseCam0 = new tf::MessageFilter<geometry_msgs::TransformStamped>(subPoseCam0, tfListener, "camera0", 10);
  tfFilterPoseCam1 = new tf::MessageFilter<geometry_msgs::TransformStamped>(subPoseCam1, tfListener, "camera1", 10);
  tfFilterPoseCam2 = new tf::MessageFilter<geometry_msgs::TransformStamped>(subPoseCam2, tfListener, "camera2", 10);
  tfFilterPoseCam3 = new tf::MessageFilter<geometry_msgs::TransformStamped>(subPoseCam3, tfListener, "camera3", 10);

  tfFilterPoseCam0->registerCallback(&PanopticonPoser::cameraPoseCallback0, this);
  tfFilterPoseCam1->registerCallback(&PanopticonPoser::cameraPoseCallback1, this);
  tfFilterPoseCam2->registerCallback(&PanopticonPoser::cameraPoseCallback2, this);
  tfFilterPoseCam3->registerCallback(&PanopticonPoser::cameraPoseCallback3, this);  
}

PanopticonPoser::~PanopticonPoser() {
  delete tfFilterPoseCam0;
  delete tfFilterPoseCam1;
  delete tfFilterPoseCam2;
  delete tfFilterPoseCam3;
}

void PanopticonPoser::cameraPoseCallbackWithCamId(const geometry_msgs::TransformStamped::ConstPtr& msg, string camId) {
  string frameName = msg->child_frame_id;
  string parentName = msg->header.frame_id;


  //if (boost::starts_with(frameName, "marker")) {
    try{
      tf::Transform transform;
      transformMsgToTF(msg->transform, transform);

      geometry_msgs::PoseStamped pose;
      geometry_msgs::PoseStamped poseInWorld;

      tf::poseTFToMsg(transform, pose.pose);
      pose.header.frame_id = msg->header.frame_id;
      pose.header.stamp = msg->header.stamp;

      // It is a hack here, as I did not find out how to properly do the
      // timing so that tf2 does not complain about requesting from past/future
      // or extrapolation needed
      pose.header.stamp = poseInWorld.header.stamp = ros::Time(0);

      tfListener.transformPose("map", pose, poseInWorld);
      geometry_msgs::PoseWithCovarianceStamped poseWorldWithCov = poseToPoseWithCovariance(poseInWorld);

      boost::unordered_map<std::string, RawPosePublisher>::iterator it = posePublishers.find(frameName);

      if(it == posePublishers.end()) {
        RawPosePublisher tmp = RawPosePublisher(nh, frameName);
        posePublishers.insert(std::make_pair<std::string,RawPosePublisher>(frameName,tmp));
      }

      RawPosePublisher rawPub = posePublishers[frameName];

      if(camId == "camera0") {
        rawPub.pubPoseCam0.publish(poseInWorld);
      } else if(camId == "camera1") {
        rawPub.pubPoseCam1.publish(poseInWorld);
      } else if(camId == "camera2") {
        rawPub.pubPoseCam2.publish(poseInWorld);
      } else if(camId == "camera3") {
        rawPub.pubPoseCam3.publish(poseInWorld);
      } else {
        ROS_ERROR("Invalid Camera Id: %s", camId.c_str());
      }

    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
}

void PanopticonPoser::cameraPoseCallback0(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  cameraPoseCallbackWithCamId(msg, "camera0");
}

void PanopticonPoser::cameraPoseCallback1(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  cameraPoseCallbackWithCamId(msg, "camera1");
}

void PanopticonPoser::cameraPoseCallback2(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  cameraPoseCallbackWithCamId(msg, "camera2");
}

void PanopticonPoser::cameraPoseCallback3(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  cameraPoseCallbackWithCamId(msg, "camera3");
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
