#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_eigen/tf2_eigen.h>


namespace pclLabelingHelpers {
    void getCameraProjection(ros::NodeHandle* nh, Eigen::Matrix<double, 3, 4>& cameraProjection);
    void getTfLidarCamera(ros::NodeHandle* nh, std::string lidarFrame, std::string cameraFrame, geometry_msgs::TransformStamped& staticLidarCamTf);
} // namespace pclLabelingHelpers
