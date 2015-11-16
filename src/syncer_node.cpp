#include "syncer_node.hpp"

#include <string>

// https://bitbucket.org/tsenlet/robotts/wiki/Stereo%20Processing%20Project%20Template
// http://docs.ros.org/diamondback/api/image_transport/html/classimage__transport_1_1SubscriberFilter.html
//
using std::string;

using sensor_msgs::ImageConstPtr;
using sensor_msgs::CameraInfoConstPtr;

namespace panopticon {
SyncerNode::SyncerNode(const ros::NodeHandle& handle, string cameraOneNs, string cameraTwoNs)
    : it(nh),
      nh(handle),
      cam1_left_img_sub(it, cameraOneNs + "/left/image_rect", 1),
      cam1_right_img_sub(it, cameraOneNs + "/right/image_rect", 1),
      cam2_left_img_sub(it, cameraTwoNs + "/left/image_rect", 1),
      cam2_right_img_sub(it, cameraTwoNs + "/right/image_rect", 1),
      cam1_left_caminfo_sub(nh, cameraOneNs + "/left/camera_info", 1),
      cam1_right_caminfo_sub(nh, cameraOneNs + "/right/camera_info", 1),
      cam2_left_caminfo_sub(nh, cameraTwoNs + "/left/camera_info", 1),
      cam2_right_caminfo_sub(nh, cameraTwoNs + "/right/camera_info", 1),
      sync(SyncPolicy(100),
           cam1_left_img_sub,
           cam1_right_img_sub,
           cam2_left_img_sub,
           cam2_right_img_sub,
           cam1_left_caminfo_sub,
           cam1_right_caminfo_sub,
           cam2_left_caminfo_sub,
           cam2_right_caminfo_sub) {
  sync.registerCallback(boost::bind(&SyncerNode::stereoCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
  combo_pub = nh.advertise<ImageCollection>("chatter", 1000);
}

SyncerNode::~SyncerNode() {}

void SyncerNode::stereoCallback(const sensor_msgs::ImageConstPtr& cam1_left_img,
                                const sensor_msgs::ImageConstPtr& cam1_right_img,
                                const sensor_msgs::ImageConstPtr& cam2_left_img,
                                const sensor_msgs::ImageConstPtr& cam2_right_img,
                                const sensor_msgs::CameraInfoConstPtr& cam1_left_info,
                                const sensor_msgs::CameraInfoConstPtr& cam1_right_info,
                                const sensor_msgs::CameraInfoConstPtr& cam2_left_info,
                                const sensor_msgs::CameraInfoConstPtr& cam2_right_info) {
  ImageCollection msg;
  msg.cam1left_img = *cam1_left_img.get();
  msg.cam1right_img = *cam1_right_img;
  msg.cam2left_img = *cam2_left_img;
  msg.cam2right_img = *cam2_right_img;
  msg.cam1left_info = *cam1_left_info;
  msg.cam1right_info = *cam1_right_info;
  msg.cam2left_info = *cam2_left_info;
  msg.cam2right_info = *cam2_right_info;
  combo_pub.publish(msg);
}
}
