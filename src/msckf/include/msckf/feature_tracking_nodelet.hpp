//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#ifndef MSCKF_FEATURE_TRACKING_NODELET_HPP
#define MSCKF_FEATURE_TRACKING_NODELET_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <cv_bridge/cv_bridge.h>
#include "msckf/ImageFeatures.h"
#include "msckf/types.hpp"

namespace feature_tracker {

struct FeatureTrack {
    uint32_t latest_image_seq;
    std::vector<cv::KeyPoint> points;
};

class FeatureTrackingNodelet : public nodelet::Nodelet {
 public:
    FeatureTrackingNodelet();
    ~FeatureTrackingNodelet();
    void onInit() override;

 private:
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void publishFeatures(const cv_bridge::CvImagePtr &cv_image,
                         const std::vector<cv::KeyPoint> &keypoints,
                         const cv::Mat &descriptors);
    std::vector<cv::DMatch> matchFeatures(std::vector<cv::KeyPoint> &keypoints,
                                          const cv::Mat &descriptors,
                                          const std::vector<cv::KeyPoint> &previous_keypoints,
                                          const cv::Mat &previous_descriptors);
    std::vector<cv::DMatch> filterMatches(const std::vector<std::vector<cv::DMatch>> &knn_matches);
    int generateFeatureId();

    // Store new keypoints in feature tracks map, and delete expired tracks
    void storeFeatureTracks(const std::vector<cv::KeyPoint> &keypoints, uint32_t image_seq);

    // Draw arrows for each feature track
    void drawFeatureTracks(cv::Mat &output_image) const;

    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    ros::Subscriber camera_info_sub;
    ros::Publisher features_pub;
    cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> sift;
    cv::Ptr<cv::BFMatcher> matcher;

    // map of currently visible features and their track
    std::map<int, FeatureTrack> feature_tracks;

    // Parameter storage
    msckf::CameraParameters camera_parameters;
    bool initialized_camera = false;
    double MATCH_RATIO = 0.65;
};

}  // namespace feature_tracker

#endif  // MSCKF_FEATURE_TRACKING_NODELET_HPP
