#ifndef PCL_LABELING_POINT_LABELER_ROS_H_
#define PCL_LABELING_POINT_LABELER_ROS_H_

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "pcl_labeling/LidarCameraProjector.hpp"
#include "pcl_labeling/PointLabeler.hpp"

class PointLabelerRos {
 public:
  PointLabelerRos(ros::NodeHandle* nh);

 private:
  void initialize();
  void pclAndImgCallback(const sensor_msgs::ImageConstPtr& img_msg,
                         const sensor_msgs::PointCloud2ConstPtr& lidar_pcl_msg);

  Eigen::Matrix3Xd preprocessPcl(pcl::PointCloud<pcl::PointXYZ>& lidarPclCloud);
  void removePtsOutsideOfRange(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                               pcl::PointCloud<pcl::PointXYZ>& cloud_out, float min_range, float max_range);
  sensor_msgs::PointCloud2 eigenToRos(const Eigen::MatrixXd& cloud, const std::string& frame_id);

  ros::NodeHandle nh_;
  std::string sensor_setup_;
  std::string end_effector_type_;
  message_filters::Subscriber<sensor_msgs::Image> rgb_img_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_pcl_sub_;
  ros::Publisher rgb_pcl_pub_;
  std::string rgb_img_topic_;
  std::string lidar_pcl_topic_;
  std::string lidar_rgb_pcl_topic_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>
      ApproxTimePolicy_;
  typedef message_filters::Synchronizer<ApproxTimePolicy_> SubSynchronizer_;
  boost::shared_ptr<SubSynchronizer_> imgLidarSubSynchronizer_;
  int approxTimePolicyQueueSize_;

  std::shared_ptr<PointLabeler> pointLabeler_;
  LidarCameraProjector projector_;

  Eigen::Matrix<double, 3, 4> cameraProjection_;
  Eigen::Matrix<double, 3, 3> cameraMatrix_;
  Eigen::Matrix<double, 5, 1> distortionCoeffs_;
  int imageWidth_;
  int imageHeight_;
  std::string cameraFrame_;
  std::string lidarFrame_;
  tf2_ros::Buffer tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  Eigen::Isometry3d lidarToCameraEigen_;
  bool isLidarToCameraTransformSet_ = false;

  std::vector<int> endEffectorLabels_;
  std::vector<int> selfLabels_;
};
#endif  // PCL_LABELING_POINT_LABELER_ROS_H_