//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#ifndef MSCKF_MSCKF_NODELET_HPP
#define MSCKF_MSCKF_NODELET_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <cv_bridge/cv_bridge.h>

#include "msckf/EKF.hpp"
#include "msckf/ImageFeatures.h"

namespace msckf {

class MsckfNodelet : public nodelet::Nodelet {
 public:
    MsckfNodelet();
    ~MsckfNodelet();
    void onInit() override;

 private:
    // Publisher helpers
    void publishStateEstimate();

    // Subscriber callbacks
    void imuCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void featuresCallback(const msckf::ImageFeaturesConstPtr &msg);

    // Set up internal library's Boost logging
    void initLogging();

    // Publisher objects
    ros::Publisher odom_pub;

    // Subscriber objects
    ros::Subscriber imu_sub;
    ros::Subscriber odom_sub;
    image_transport::Subscriber image_sub;
    ros::Subscriber features_sub;

    // EKF objects
    EKF ekf{{0,0,0}, {0, 0, 0}};
};

}  // namespace msckf

#endif  // MSCKF_MSCKF_NODELET_HPP
