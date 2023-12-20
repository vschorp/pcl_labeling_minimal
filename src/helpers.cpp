#include "pcl_labeling/helpers.hpp"

void pclLabelingHelpers::getCameraProjection(ros::NodeHandle* nh, Eigen::Matrix<double, 3, 4>& cameraProjection) {
    double fx, fy, cx, cy;
    nh->param<double>("/pcl_labeling_node/pcl_labeling/camera_calibration/camera_matrix/fx", fx, 0.0);
    nh->param<double>("/pcl_labeling_node/pcl_labeling/camera_calibration/camera_matrix/fy", fy, 0.0);
    nh->param<double>("/pcl_labeling_node/pcl_labeling/camera_calibration/camera_matrix/cx", cx, 0.0);
    nh->param<double>("/pcl_labeling_node/pcl_labeling/camera_calibration/camera_matrix/cy", cy, 0.0);
    std::cout << "fx: " << fx << std::endl;

    cameraProjection <<     fx,     0.0,    cx,     0.0,
                            0.0,    fy,     cy,     0.0,
                            0.0,    0.0,    1.0,    0.0;
}

void pclLabelingHelpers::getTfLidarCamera(ros::NodeHandle* nh, std::string lidarFrame, std::string cameraFrame, geometry_msgs::TransformStamped& staticLidarCamTf) {
    double x, y, z, qx, qy, qz, qw;
    nh->param<double>("/pcl_labeling_node/pcl_labeling/tf_lidar_camera/translation/x", x, 0.0);
    nh->param<double>("/pcl_labeling_node/pcl_labeling/tf_lidar_camera/translation/y", y, 0.0);
    nh->param<double>("/pcl_labeling_node/pcl_labeling/tf_lidar_camera/translation/z", z, 0.0);
    nh->param<double>("/pcl_labeling_node/pcl_labeling/tf_lidar_camera/rotation/x", qx, 0.0);
    nh->param<double>("/pcl_labeling_node/pcl_labeling/tf_lidar_camera/rotation/y", qy, 0.0);
    nh->param<double>("/pcl_labeling_node/pcl_labeling/tf_lidar_camera/rotation/z", qz, 0.0);
    nh->param<double>("/pcl_labeling_node/pcl_labeling/tf_lidar_camera/rotation/w", qw, 0.0);
    staticLidarCamTf.header.stamp = ros::Time::now();
    staticLidarCamTf.header.frame_id = lidarFrame;
    staticLidarCamTf.child_frame_id = cameraFrame;
    staticLidarCamTf.transform.translation.x = x;
    staticLidarCamTf.transform.translation.y = y;
    staticLidarCamTf.transform.translation.z = z;
    staticLidarCamTf.transform.rotation.x = qx;
    staticLidarCamTf.transform.rotation.y = qy;
    staticLidarCamTf.transform.rotation.z = qz;
    staticLidarCamTf.transform.rotation.w = qw;
}
