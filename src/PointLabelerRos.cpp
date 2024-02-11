#include "pcl_labeling/PointLabelerRos.hpp"

PointLabelerRos::PointLabelerRos(ros::NodeHandle* nh) : nh_(*nh) { initialize(); }

void PointLabelerRos::initialize() {
  listener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_);

  // Load parameters
  nh_.getParam("/sensor_setup", sensor_setup_);
  nh_.getParam("/end_effector_type", end_effector_type_);

  // Get camera projection matrix
  double fx, fy, cx, cy;
  nh_.getParam("/intrinsic_camera_calibration/" + sensor_setup_ + "/camera_matrix/fx", fx);
  nh_.getParam("/intrinsic_camera_calibration/" + sensor_setup_ + "/camera_matrix/fy", fy);
  nh_.getParam("/intrinsic_camera_calibration/" + sensor_setup_ + "/camera_matrix/cx", cx);
  nh_.getParam("/intrinsic_camera_calibration/" + sensor_setup_ + "/camera_matrix/cy", cy);
  cameraProjection_ << fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0;
  projector_.setCameraProjectionMatrix(cameraProjection_);

  nh_.getParam("/frames/lidar_frame", lidarFrame_);
  nh_.getParam("/frames/camera_frame", cameraFrame_);
  std::cout << "Lidar Frame is: " << lidarFrame_ << std::endl;
  std::cout << "Camera Frame is: " << cameraFrame_ << std::endl;

  nh_.getParam("/topics/preprocessed_img_topic", rgb_img_topic_);
  nh_.getParam("/topics/lidar_pcl_topic", lidar_pcl_topic_);
  nh_.getParam("/topics/lidar_rgb_pcl_topic", lidar_rgb_pcl_topic_);
  std::cout << "rgb_img_topic_ is: " << rgb_img_topic_ << std::endl;
  std::cout << "lidar_pcl_topic_ is: " << lidar_pcl_topic_ << std::endl;
  std::cout << "lidar_rgb_pcl_topic is: " << lidar_rgb_pcl_topic_ << std::endl;

  pointLabeler_.reset(new PointLabeler(projector_));

  rgb_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(lidar_rgb_pcl_topic_, 1);

  rgb_img_sub_.subscribe(nh_, rgb_img_topic_, 1);
  lidar_pcl_sub_.subscribe(nh_, lidar_pcl_topic_, 1);

  imgLidarSubSynchronizer_.reset(
      new SubSynchronizer_(PointLabelerRos::ApproxTimePolicy_(10), rgb_img_sub_, lidar_pcl_sub_));
  imgLidarSubSynchronizer_->registerCallback(boost::bind(&PointLabelerRos::pclAndImgCallback, this, _1, _2));
}

Eigen::Matrix3Xd PointLabelerRos::preprocessPcl(pcl::PointCloud<pcl::PointXYZ>& lidarPclCloud) {
  float min_range = 0.1;  // in meters
  float max_range = 7.0;  // in meters
  removePtsOutsideOfRange(lidarPclCloud, lidarPclCloud, min_range, max_range);
  Eigen::Matrix3Xd lidarPclCloudMatrix = lidarPclCloud.getMatrixXfMap().cast<double>();
  return lidarPclCloudMatrix;
}

void PointLabelerRos::pclAndImgCallback(const sensor_msgs::ImageConstPtr& img_msg,
                                        const sensor_msgs::PointCloud2ConstPtr& lidar_pcl_msg) {
  std::cout << "Received image and point cloud" << std::endl;
  if (!isLidarToCameraTransformSet_) {
    geometry_msgs::TransformStamped lidarToCamera;
    try {
      lidarToCamera =
          tfBuffer_.lookupTransform(cameraFrame_, lidarFrame_, ros::Time(0));  // Time(0) gets latest transform
    } catch (tf2::LookupException& e) {
      ROS_ERROR("%s", e.what());
      return;
    }
    lidarToCameraEigen_ = tf2::transformToEigen(lidarToCamera);
    isLidarToCameraTransformSet_ = true;
  }
  cv::Mat img;
  assert(img_msg->encoding == "bgr8");
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  img = cv_ptr->image;

  pcl::PointCloud<pcl::PointXYZ> lidarPclCloud;
  pcl::fromROSMsg(*lidar_pcl_msg, lidarPclCloud);
  Eigen::Matrix3Xd lidarPclCloudMatrix = preprocessPcl(lidarPclCloud);

  pointLabeler_->processPointCloud(lidarPclCloudMatrix, img, lidarToCameraEigen_);

  Eigen::MatrixXd rgbPointCloudMatrix = pointLabeler_->getRGBPointCloud();
  sensor_msgs::PointCloud2 rgbPointCloudMsg = eigenToRos(rgbPointCloudMatrix, lidarFrame_);

  rgb_pcl_pub_.publish(rgbPointCloudMsg);
}

void PointLabelerRos::removePtsOutsideOfRange(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                                              pcl::PointCloud<pcl::PointXYZ>& cloud_out, float min_range,
                                              float max_range) {
  std::size_t j = 0;
  for (std::size_t i = 0; i < cloud_in.size(); ++i) {
    auto dist =
        std::sqrt(cloud_in[i].x * cloud_in[i].x + cloud_in[i].y * cloud_in[i].y + cloud_in[i].z * cloud_in[i].z);
    if (dist > max_range || dist < min_range) {
      continue;
    }
    cloud_out[j] = cloud_in[i];
    j++;
  }
  if (j != cloud_out.size()) {
    cloud_out.resize(j);
  }
  cloud_out.height = 1;
  cloud_out.width = static_cast<std::uint32_t>(j);
}

sensor_msgs::PointCloud2 PointLabelerRos::eigenToRos(const Eigen::MatrixXd& cloud, const std::string& frame_id) {
  using namespace sensor_msgs;
  PointCloud2 outCloud;
  outCloud.header.frame_id = frame_id;
  outCloud.header.stamp = ros::Time::now();
  outCloud.height = 1;
  outCloud.width = cloud.cols();

  PointCloud2Modifier modifier(outCloud);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32, "rgb", 1, sensor_msgs::PointField::FLOAT32);

  PointCloud2Iterator<float> x_iter(outCloud, "x");
  PointCloud2Iterator<float> y_iter(outCloud, "y");
  PointCloud2Iterator<float> z_iter(outCloud, "z");
  PointCloud2Iterator<uint8_t> r_iter(outCloud, "r");
  PointCloud2Iterator<uint8_t> g_iter(outCloud, "g");
  PointCloud2Iterator<uint8_t> b_iter(outCloud, "b");

  for (int i = 0; i < cloud.cols(); ++i, ++x_iter, ++y_iter, ++z_iter, ++r_iter, ++g_iter, ++b_iter) {
    const Eigen::VectorXd& point = cloud.col(i);
    *x_iter = point(0);
    *y_iter = point(1);
    *z_iter = point(2);
    *r_iter = static_cast<uchar>(point(3));
    *g_iter = static_cast<uchar>(point(4));
    *b_iter = static_cast<uchar>(point(5));
  }
  return outCloud;
}