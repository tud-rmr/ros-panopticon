#pragma once

#include <string>

#include <ros/ros.h>

#include <tf/tf.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>

#include <boost/circular_buffer.hpp>

#include <std_srvs/Empty.h>

namespace rmr {
class PanopticonTransformer {
 public:
  PanopticonTransformer(ros::NodeHandle _nh);
  ~PanopticonTransformer();

  void waitForWorldFrame(const std::string& topic);

 private:
  ros::NodeHandle nh;

  // Ros services
  bool set_cam_transforms_callback(std_srvs::Empty::Request& request,
                                          std_srvs::Empty::Response& response);
  ros::ServiceServer set_cam_transforms_srv;

  /* Transform subs */

  ros::Subscriber subTransformCam0;
  ros::Subscriber subTransformCam1;
  ros::Subscriber subTransformCam2;
  ros::Subscriber subTransformCam3;

  tf::TransformBroadcaster tfBroadcaster;

  geometry_msgs::TransformStamped origin;

  std::string frameContainingMap;

  // Circular buffers for raw tansformations
  boost::circular_buffer<tf::StampedTransform> cam_to_center_buffer[4];
  boost::circular_buffer<tf::StampedTransform> mainCam_to_map_buffer;

  // Final filtered transformations
  tf::StampedTransform map_to_cam[4];

  /* Callbacks */
  void cameraTransformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);

  /* Helper methods */
  void publishInversedTransformation(const geometry_msgs::TransformStamped::ConstPtr& msg);

  /*tf interpolation */
  tf::StampedTransform& tf_interpolation(tf::StampedTransform& new_tf, tf::StampedTransform& old_tf, double interpolation_weight);
  tf::Vector3 vector_interpolation(tf::Vector3 new_vector, tf::Vector3 old_vector, double interpolation_weight);
  tf::Quaternion quaternion_interpolation(tf::Quaternion new_q, tf::Quaternion old_q, double interpolation_weight);
  tf::StampedTransform do_interpolation_tf_buffer(boost::circular_buffer<tf::StampedTransform> buffer);
  void calculate_fixed_tf(void);
};
}
