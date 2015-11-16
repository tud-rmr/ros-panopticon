#pragma once

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <panopticon/ImageCollection.h>

namespace panopticon {
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo,
                                                        sensor_msgs::CameraInfo,
                                                        sensor_msgs::CameraInfo,
                                                        sensor_msgs::CameraInfo> SyncPolicy;

class SyncerNode {
 public:
  SyncerNode(const ros::NodeHandle& handle, std::string, std::string);
  ~SyncerNode();

  void stereoCallback(const sensor_msgs::ImageConstPtr& cam1_left_img,
                      const sensor_msgs::ImageConstPtr& cam1_right_img,
                      const sensor_msgs::ImageConstPtr& cam2_left_img,
                      const sensor_msgs::ImageConstPtr& cam2_right_img,
                      const sensor_msgs::CameraInfoConstPtr& cam1_left_info,
                      const sensor_msgs::CameraInfoConstPtr& cam1_right_info,
                      const sensor_msgs::CameraInfoConstPtr& cam2_left_info,
                      const sensor_msgs::CameraInfoConstPtr& cam2_right_info);

 private:
  ros::NodeHandle nh;
  ros::Publisher combo_pub;
  image_transport::ImageTransport it;

  image_transport::SubscriberFilter cam1_left_img_sub;
  image_transport::SubscriberFilter cam1_right_img_sub;
  image_transport::SubscriberFilter cam2_left_img_sub;
  image_transport::SubscriberFilter cam2_right_img_sub;

  message_filters::Subscriber<sensor_msgs::CameraInfo> cam1_left_caminfo_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam1_right_caminfo_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam2_left_caminfo_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam2_right_caminfo_sub;

  message_filters::Synchronizer<SyncPolicy> sync;
};
}
