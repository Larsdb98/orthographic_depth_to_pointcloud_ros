# Orthographic Depth Map to Point Cloud ROS

ROS distro: Noetic (Ubuntu 20.04)

This ROS package computes point clouds (sensor_msgs/PointCloud2) from orthographic top-view depth maps (sensor_msgs/Image). Since the depth maps come from orthographic projections, the camera intrinsics are not needed. However this package assumes that the center of the camera frame is at the center of the depth maps (i.e. [height/2, width/2]).

The published point clouds have the same frame ID and timestamp as the depoth images.

## Parameters

- **~ortho_width** : The physical width of the orthographic depth image (in meters).
- **~depth_image_topic** : Topic where orthographic depth images are being published.
- **~pointcloud_out_topic** : Topic where the sensor_msgs/Pointcloud2 messages should be published. 

## Example

See the [demo.launch](launch/demo.launch) launchfile.

## Installation

1. To install, clone this repository into your catkin ```src``` folder with the following command lines:
    ```bash
    cd ~/catkin_ws/src
    git clone git@github.com:Larsdb98/orthographic_depth_to_pointcloud_ros.git
    ```

2. Build and source your workspace:
    ```bash
    cd ~/catkin_ws
    catkin build
    source devel/setup.bash
    ```

3. Launch the node:
    ```bash
    roslaunch orthographic_depth_to_pointcloud_ros demo.launch
    ```

Build and source your workspace




# License

This project is licensed under the MIT license - see the [LICENSE.md](LICENSE.md) file for details.
