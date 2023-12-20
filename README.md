# pcl_labeling_minimal
Minimal Point Cloud Labeling Example based on the `leggedrobotics/rsl_dynamic_mapping` repo by Pol Eyschen.

## Setup
This repo contains a submodule so use the `--recusive` flag when cloning the repo. If you already cloned the repo you can use `git submodule init` followed by `git submodule update`.

## Structure
- The ROS Node is started in `pcl_labeling_node.cpp`.
- All ROS related data handling is done in the `PointLabelerRos` class. Most required parameters (topic names, camera to lidar calibration, ...) are set in the `config/param.yaml`. 
- The `PointLabeler` class contains the main functionality whereas the the `LidarCameraProjector` handles the projection of the lidar points to the camera image.

## Usage
- The node requires a point cloud and an undistorted RGB image as input and returns a cloud whos points are colored by the RGB values of the image.
- To perform the projection of the point cloud to the image, the lidar to camera extrinsic calibration needs to be known beforehand. This can be found using the [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration.git) repo or something similar. 
- to build the project, it is recommended to first set 
```$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo```
- If the build fails with some `Eigen` error, the compiler probably didn't take the correct Eigen version from your computer. Check that you have pulled the submodule.
- Once built, the node can be started with
```$ roslaunch pcl_labeling pcl_labeling.launch``` 
