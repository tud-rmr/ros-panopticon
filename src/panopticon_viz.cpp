#include "panopticon_viz.hpp"

#include <ros/topic.h>

#include <boost/algorithm/string/predicate.hpp>

using std::string;

namespace rmr {

PanopticonViz::PanopticonViz(ros::NodeHandle _nh) {
  nh = _nh;

  subPoseCam0.subscribe(nh, "camera0/pose", 10);
  subPoseCam1.subscribe(nh, "camera1/pose", 10);
  subPoseCam2.subscribe(nh, "camera2/pose", 10);
  subPoseCam3.subscribe(nh, "camera3/pose", 10);

  tfFilterPoseCam0 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam0, tfListener, "usb_cam0", 10);
  tfFilterPoseCam1 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam1, tfListener, "usb_cam1", 10);
  tfFilterPoseCam2 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam2, tfListener, "usb_cam2", 10);
  tfFilterPoseCam3 = new tf::MessageFilter<geometry_msgs::PoseStamped>(subPoseCam3, tfListener, "usb_cam3", 10);

  tfFilterPoseCam0->registerCallback(&PanopticonViz::cameraPoseCallback, this);
  tfFilterPoseCam1->registerCallback(&PanopticonViz::cameraPoseCallback, this);
  tfFilterPoseCam2->registerCallback(&PanopticonViz::cameraPoseCallback, this);
  tfFilterPoseCam3->registerCallback(&PanopticonViz::cameraPoseCallback, this);

  pubPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1000);
}

PanopticonViz::~PanopticonViz() {
  delete tfFilterPoseCam0;
  delete tfFilterPoseCam1;
  delete tfFilterPoseCam2;
  delete tfFilterPoseCam3;
}

void PanopticonViz::cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

}

}