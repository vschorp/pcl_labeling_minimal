#include "pcl_labeling/PointLabeler.hpp"

void PointLabeler::processPointCloud(const Eigen::Matrix3Xd& pts, const cv::Mat& segMask,
                                     const Eigen::Isometry3d& lidarToCamera) {
  // Here segMask stands for image as this class was initially written for using 2D segmentation masks instead of rgb
  // images.
  std::vector<int> indices;
  Eigen::MatrixXd coloredStaticCloud = labelPoints(pts, segMask, lidarToCamera, indices);
  rgbPointCloud_.conservativeResize(6, pts.cols());
  rgbPointCloud_.topRows(3) = pts;
  rgbPointCloud_.bottomRows(3).setConstant(255);

  for (int i = 0; i < indices.size(); i++) {
    auto red = coloredStaticCloud(3, i);
    auto green = coloredStaticCloud(4, i);
    auto blue = coloredStaticCloud(5, i);

    rgbPointCloud_(3, indices[i]) = red;
    rgbPointCloud_(4, indices[i]) = green;
    rgbPointCloud_(5, indices[i]) = blue;
  }
}

Eigen::MatrixXd PointLabeler::labelPoints(const Eigen::Matrix3Xd& pts, const cv::Mat& segMask,
                                          const Eigen::Isometry3d& lidarToCamera, std::vector<int>& labelPtsIdx) {
  projector_.projectPtsToCameraFrame(pts, lidarToCamera);
  labelPtsIdx = projector_.filterVisiblePoints(pts, segMask.size());
  Eigen::Matrix3Xi pixelPts = projector_.getPixelPoints().cast<int>();
  Eigen::Matrix3Xd visiblePts = projector_.getVisiblePoints();

  Eigen::RowVectorXd red(pixelPts.cols());
  Eigen::RowVectorXd green(pixelPts.cols());
  Eigen::RowVectorXd blue(pixelPts.cols());

  for (int i = 0; i < pixelPts.cols(); i++) {
    cv::Vec3b pixel_bgr = segMask.at<cv::Vec3b>(cv::Point(pixelPts(0, i), pixelPts(1, i)));
    blue(i) = static_cast<double>(pixel_bgr[0]);
    green(i) = static_cast<double>(pixel_bgr[1]);
    red(i) = static_cast<double>(pixel_bgr[2]);
  }

  Eigen::MatrixXd coloredPoints(6, visiblePts.cols());
  coloredPoints << visiblePts, red, green, blue;
  return coloredPoints;
}
