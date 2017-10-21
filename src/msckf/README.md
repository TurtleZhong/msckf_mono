# MSCKF

Experimental implementation of the Multi-State Constraint Kalman Filter with ROS interface.

Based on the paper by Mourikis and Roumeliotis ([PDF](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.125.2169&rep=rep1&type=pdf)).

Also includes a feature tracking node using OpenCV's SIFT implementation.

This is a work in progress. In particular, The API is a bit non-standard and is subject to change.

## ROS nodes

### feature_tracking_node
#### Subscribed topics

- `camera/image_rect` (sensor_msgs/Image) - Undistorted image stream.
- `camera/camera_info` (sensor_msgs/CameraInfo) - The corresponding camera metadata.


#### Published topics

- `output_image` (sensor_msgs/Image) - Image with arrows drawn on it, showing matches.
- `features` (msckf/ImageFeatures) - custom message describing feature positions

### msckf_node
#### Subscribed topics
- `imu_vel` (geometry_msgs/TwistStamped) - IMU measurement of linear and angular velocity.
- `odom` (nav_msgs/Odometry) - Position of robot, used to initialize the filter
- `camera/image_rect` (sensor_msgs/Image)-
the same image stream used by `feature_tracking_node`. Currently, only the header information is used.
- `features` (msckf/ImageFeatures) - the feature information published by `feature_tracking_node`

#### Published topics
- `odom_combined` (geometry_msgs/PoseWithCovarianceStamped) - Estimated robot pose