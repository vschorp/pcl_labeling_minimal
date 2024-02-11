#ifndef PCL_LABELING_POINT_LABELER_H_
#define PCL_LABELING_POINT_LABELER_H_

// #include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <iostream>
#include <set>
#include <vector>

#include "pcl_labeling/LidarCameraProjector.hpp"

class PointLabeler {
 public:
  PointLabeler(LidarCameraProjector& projector) : projector_(projector) {}
  void processPointCloud(const Eigen::Matrix3Xd& pts, const cv::Mat& segMask, const Eigen::Isometry3d& lidarToCamera);
  Eigen::MatrixXd labelPoints(const Eigen::Matrix3Xd& pts, const cv::Mat& segMask,
                              const Eigen::Isometry3d& lidarToCamera, std::vector<int>& labelPtsIdx);
  Eigen::MatrixXd getRGBPointCloud() { return rgbPointCloud_; }

 private:
  LidarCameraProjector& projector_;
  // Eigen::Matrix4Xd labelledPoints_;
  // bool hasNewLabels_ = false;
  Eigen::MatrixXd rgbPointCloud_;
};
#endif  // PCL_LABELING_POINT_LABELER_H_