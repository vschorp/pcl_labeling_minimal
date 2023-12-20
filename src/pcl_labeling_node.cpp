#include <ros/ros.h>
#include "pcl_labeling/PointLabelerRos.hpp"


int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_labeling_node");
  ros::NodeHandle nh;

  PointLabelerRos pointLabelerRos(&nh);
  ROS_INFO("Started PCL Labeler ndoe");
  ros::spin();
}

