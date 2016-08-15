#include "panopticon_transformer.hpp"

#include <ros/topic.h>

#include <boost/algorithm/string/predicate.hpp>

using std::string;

namespace rmr {

PanopticonTransformer::PanopticonTransformer(ros::NodeHandle _nh) {
  nh = _nh;

  //!TODO(racuna) this should be a rosparam
  frameContainingMap = "camera2";

  //!TODO(racuna) this topic subscriptions should be as well a ros param
  subTransformCam0 = nh.subscribe("camera0/transform", 1000, &PanopticonTransformer::cameraTransformCallback, this);
  subTransformCam1 = nh.subscribe("camera1/transform", 1000, &PanopticonTransformer::cameraTransformCallback, this);
  subTransformCam2 = nh.subscribe("camera2/transform", 1000, &PanopticonTransformer::cameraTransformCallback, this);
  subTransformCam3 = nh.subscribe("camera3/transform", 1000, &PanopticonTransformer::cameraTransformCallback, this);

  set_cam_transforms_srv = nh.advertiseService("panopticon/set_cam_transforms", &PanopticonTransformer::set_cam_transforms_callback, this);

  // set identity for map to cam transformations
  for(int i=0; i<4; i++){
    map_to_cam[i].setIdentity();
    map_to_cam[i].frame_id_ = "map";
    map_to_cam[i].child_frame_id_ = "camera"+i;
    map_to_cam[i].stamp_ = ros::Time::now();
  }

  int camera_framerate;
  nh.param<int>("camera_framerate", camera_framerate, 30); // in hz

  for(int i=0; i<4; i++){
    cam_to_center_buffer[i].set_capacity(camera_framerate*5);
  }
  mainCam_to_map_buffer.set_capacity(camera_framerate*5);
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

  tf::StampedTransform st;
  //st.stamp_ = ros::Time::now();
  tf::transformStampedMsgToTF(*msg, st);

  if(frameName == "map" || frameName == "center"){
    if(frameName == "map" && parentName == frameContainingMap) {
      //Store value in buffer
      mainCam_to_map_buffer.push_back(st);
    } else if (frameName == "center") {

      if(parentName == "camera0") {
        cam_to_center_buffer[0].push_back(st);
      } else if(parentName == "camera1"){
        cam_to_center_buffer[1].push_back(st);
      } else if(parentName == "camera2"){
        cam_to_center_buffer[2].push_back(st);
      } else if(parentName == "camera3"){
        cam_to_center_buffer[3].push_back(st);
      }

      }
    }

  // Publish all the transformations
  for(int i=0; i<4; i++){
    tfBroadcaster.sendTransform(map_to_cam[i]);
  }

}

void PanopticonTransformer::calculate_fixed_tf(void){
  //Check all the buffers if they have valid data
  tf::Transform mainCam_to_map;
  if(!mainCam_to_map_buffer.empty()){
     mainCam_to_map = do_interpolation_tf_buffer(mainCam_to_map_buffer);
  }else{
    mainCam_to_map.setIdentity();
  }


  tf::StampedTransform cam_to_center[4];
  tf::Transform mainCam_to_center;
  for(int i=0; i<4; i++){
    if(!cam_to_center_buffer[i].empty()){
       cam_to_center[i] = do_interpolation_tf_buffer(cam_to_center_buffer[i]);
    }else{
      cam_to_center[i].setIdentity();
    }
    cam_to_center[i].frame_id_ = "camera"+i;
    cam_to_center[i].child_frame_id_ = "center";
    cam_to_center[i].stamp_ = ros::Time::now();

    if(cam_to_center[i].frame_id_ == frameContainingMap){
      mainCam_to_center = cam_to_center[i];
    }
  }


  // map to cam trasnformations

  for(int i=0; i<4; i++){
    if(cam_to_center[i].frame_id_ != frameContainingMap){
      map_to_cam[i].setData(mainCam_to_map.inverse()*mainCam_to_center*cam_to_center[i].inverse());
    }else{
      map_to_cam[i].setData(mainCam_to_map.inverse());
    }
    map_to_cam[i].frame_id_ = "map";
    map_to_cam[i].child_frame_id_ = "camera"+i;
    map_to_cam[i].stamp_ = ros::Time::now();
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


tf::StampedTransform& PanopticonTransformer::tf_interpolation(tf::StampedTransform& new_tf,
                                           tf::StampedTransform& old_tf, double interpolation_weight) {

  tf::Vector3 pos = vector_interpolation(new_tf.getOrigin(), old_tf.getOrigin(), interpolation_weight);
  new_tf.setOrigin(pos);

  tf::Quaternion orn = quaternion_interpolation(new_tf.getRotation(), old_tf.getRotation(), interpolation_weight);
  new_tf.setRotation(orn);
  return new_tf;
}

tf::Vector3 PanopticonTransformer::vector_interpolation(
    tf::Vector3 new_vector, tf::Vector3 old_vector,
    double interpolation_weight) {
  tf::Vector3 interpolated_vector((1 - interpolation_weight) * old_vector.x() +
                                       interpolation_weight * new_vector.x(),
                                  (1 - interpolation_weight) * old_vector.y() +
                                       interpolation_weight * new_vector.y(),
                                  (1 - interpolation_weight) * old_vector.z() +
                                       interpolation_weight * new_vector.z());
  return interpolated_vector;
}

tf::Quaternion PanopticonTransformer::quaternion_interpolation(
    tf::Quaternion new_q, tf::Quaternion old_q,
    double interpolation_weight){
    tf::Quaternion interpolated_q;
    interpolated_q = old_q.slerp(new_q, interpolation_weight);
    return interpolated_q;
  }

tf::StampedTransform PanopticonTransformer::do_interpolation_tf_buffer(boost::circular_buffer<tf::StampedTransform> buffer){

  tf::StampedTransform tf_filtered;
  tf::StampedTransform tf_input = buffer[0];

  int count = 1.0;
  tf_filtered.setIdentity();

  for(tf::StampedTransform it : buffer){

    tf_filtered = tf_interpolation( it, tf_filtered, (1.0/count));
    count++;
    //ROS_INFO("X in each tf: %f, filtered: %f, count %d", it.getOrigin().x(), tf_filtered.getOrigin().x(), count);
  }

  tf_filtered.frame_id_ = tf_input.frame_id_;
  tf_filtered.child_frame_id_ = tf_input.child_frame_id_;
  tf_filtered.stamp_ = ros::Time::now();

  return tf_filtered;
}


//implement a service that waits and makes an average of transformations.
// store transformation from camera to center and from camera to map in a circular buffer (length to be defined)
// when the service requires an average use the information from the circular buffers and publish a fix transformation.


bool PanopticonTransformer::set_cam_transforms_callback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response){
  calculate_fixed_tf();
  // Publish all the transformations
  for(int i=0; i<4; i++){
    tfBroadcaster.sendTransform(map_to_cam[i]);
  }
  return true;
}

}
