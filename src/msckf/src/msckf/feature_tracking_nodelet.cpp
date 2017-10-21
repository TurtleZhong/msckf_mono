//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//


#include "msckf/feature_tracking_nodelet.hpp"

#include <pluginlib/class_list_macros.h>
#include "msckf/util.hpp"

PLUGINLIB_EXPORT_CLASS(feature_tracker::FeatureTrackingNodelet,
                       nodelet::Nodelet);

namespace feature_tracker {

FeatureTrackingNodelet::FeatureTrackingNodelet() {
    sift = cv::xfeatures2d::SiftFeatureDetector::create();
    matcher = cv::BFMatcher::create();

}

FeatureTrackingNodelet::~FeatureTrackingNodelet() {

}

void FeatureTrackingNodelet::onInit() {
    auto nh = getNodeHandle();
    auto image_transport = image_transport::ImageTransport{getNodeHandle()};

    features_pub = nh.advertise<msckf::ImageFeatures>("features", 10);
    image_pub = image_transport.advertise("output_image", 1);
    image_sub = image_transport.subscribe("/camera/image_rect", 3,
                                          &FeatureTrackingNodelet::imageCallback, this);
    camera_info_sub = nh.subscribe("/camera/camera_info", 1,
                                   &FeatureTrackingNodelet::cameraInfoCallback, this);
}

void FeatureTrackingNodelet::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    static cv::Mat previous_descriptors;
    static std::vector<cv::KeyPoint> previous_keypoints;

    auto cv_image_p = cv_bridge::CvImagePtr{};
    try {
        cv_image_p = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception &e) {
        NODELET_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }

    auto &image = cv_image_p->image;
    auto keypoints = std::vector<cv::KeyPoint>{};
    auto descriptors = cv::Mat{};

    // Mask out middle of the image
    const auto MASK_W = image.cols / 3;
    const auto MASK_H = image.rows / 3;
    cv::Mat mask = cv::Mat::ones(image.rows, image.cols, CV_8U);
    mask(cv::Rect(image.cols/2 - MASK_W/2, image.rows/2 - MASK_H/2, MASK_W, MASK_H)) = 0;


    sift->detectAndCompute(image, mask, keypoints, descriptors);

    NODELET_DEBUG_STREAM("Detected " << keypoints.size() << " keypoints in image");

    // Assign IDs to keypoints
    for (auto &k : keypoints) {
        k.class_id = generateFeatureId();
    }

    auto matches = matchFeatures(keypoints, descriptors, previous_keypoints, previous_descriptors);

    publishFeatures(cv_image_p, keypoints, descriptors);

    cv::Mat output = image;

    // Store all features in feature tracks map
    // Any without a match next pass will be pruned
    storeFeatureTracks(keypoints, cv_image_p->header.seq);


    // Draw a circle for each current keypoint
    cv::drawKeypoints(image, keypoints, image, -1,
                      cv::DrawMatchesFlags::DRAW_OVER_OUTIMG/* | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS*/);

    // Draw arrows showing motion of matched features
    drawFeatureTracks(output);

    image_pub.publish(cv_image_p->toImageMsg());

    previous_descriptors = descriptors;
    previous_keypoints = keypoints;
}

void FeatureTrackingNodelet::drawFeatureTracks(cv::Mat &output_image) const {
    auto drawn = 0;
    auto longest_track = 0ul;

    for (const auto &ft : feature_tracks) {
        const auto &points = ft.second.points;
        // change color based on track length
        auto color = cv::Scalar(255, 255, 0, 1);  // yellow
        auto i = points.size();
        if (i > 5) {
            color = cv::Scalar(50, 255, 50, 1);  // green
        } else {
            break;
        }

        for (i = 1; i < points.size(); ++i) {
            auto &curr = points[i];
            auto &prev = points[i - 1];
            arrowedLine(output_image, prev.pt, curr.pt, color);
        }
        if (points.size() > longest_track) {
            longest_track = points.size();
        }
        ++drawn;
    }

    NODELET_INFO_STREAM("Drew " << drawn << " tracks, the longest was: " << longest_track);
}

void FeatureTrackingNodelet::storeFeatureTracks(const std::vector<cv::KeyPoint> &keypoints,
                                                uint32_t image_seq) {
    for (const auto &k : keypoints) {
        const auto &feature_id = k.class_id;

        // Either create or update existing entry
        feature_tracks[feature_id].latest_image_seq = image_seq;
        feature_tracks[feature_id].points.push_back(k);
    }

    // Delete outdated matches from feature tracks map
    for (auto it = feature_tracks.cbegin(); it != feature_tracks.cend();) {
        const auto &track = it->second;
        if (track.latest_image_seq != image_seq) {
            // This feature does not exist in the latest frame. Erase this track from the map
            it = feature_tracks.erase(it);
        } else {
            ++it;
        }
    }
}

void FeatureTrackingNodelet::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
    /* Store camera parameters from intrinsic matrix K, which looks like
     *     [fx  0 cx]
     * K = [ 0 fy cy]
     *     [ 0  0  1]
     * and is row-major.
     */
    camera_parameters.fx = msg->K[0];
    camera_parameters.fy = msg->K[4];
    camera_parameters.cx = msg->K[2];
    camera_parameters.cy = msg->K[5];

    if (!initialized_camera) {
        NODELET_INFO_STREAM("Received camera intrinsics: " << camera_parameters.fx << ", "
                                                           << camera_parameters.fy << ", " << camera_parameters.cx
                                                           << ", " << camera_parameters.cy);

        initialized_camera = true;
    }
}

void FeatureTrackingNodelet::publishFeatures(const cv_bridge::CvImagePtr &cv_image,
                                             const std::vector<cv::KeyPoint> &keypoints,
                                             const cv::Mat &descriptors) {
    // Need camera parameters to publish idealized measurements
    if (!initialized_camera) {
        NODELET_WARN_THROTTLE(5, "Have not received CameraInfo. Not publishing features.");
    }

    msckf::ImageFeatures msg;
    msg.image_seq = cv_image->header.seq;

    int i = 0;
    for (const auto &k : keypoints) {
        msckf::Feature f;

        // Idealize the measurement
        auto ideal_pos = msckf::idealizeMeasurement({f.position.x, f.position.y}, camera_parameters);

        f.position.x = ideal_pos.x();
        f.position.y = ideal_pos.y();
        f.id = k.class_id;
        f.angle = k.angle;
        f.octave = k.octave;
        f.size = k.size;
        msg.features.push_back(f);
        ++i;
    }

    features_pub.publish(msg);
}

std::vector<cv::DMatch>
FeatureTrackingNodelet::matchFeatures(std::vector<cv::KeyPoint> &keypoints,
                                      const cv::Mat &descriptors,
                                      const std::vector<cv::KeyPoint> &previous_keypoints,
                                      const cv::Mat &previous_descriptors) {
    assert(keypoints.size() == descriptors.rows);
    assert(previous_keypoints.size() == previous_descriptors.rows);

    auto knn_matches = std::vector<std::vector<cv::DMatch>>{};
    if (descriptors.rows > 0 && previous_descriptors.rows > 0) {
        // Prepare mask with pixel distance threshold
        const int MAX_MOVEMENT_PX = 200;
        auto mask = cv::Mat(descriptors.rows, previous_descriptors.rows, CV_8U);
        for (std::size_t i = 0; i < mask.rows; ++i) {
            auto p = mask.ptr<uint8_t>(i);
            for (std::size_t j = 0; j < mask.cols; ++j) {
                const auto pixel_distance = cv::norm(keypoints[i].pt - previous_keypoints[j].pt);
                // Mask out keypoint pairs more distant than the threshold
                p[j] = uint8_t(pixel_distance < MAX_MOVEMENT_PX);
            }
        }
        // Find 2 best matches for each descriptor
        matcher->knnMatch(descriptors, previous_descriptors, knn_matches, 2, mask);
    }
    // Keep only good matches
    auto matches = filterMatches(knn_matches);

    // Assign previous keypoint ID for each match
    for (const auto &m : matches) {
        keypoints[m.queryIdx].class_id = previous_keypoints[m.trainIdx].class_id;
    }

    return matches;
}

std::vector<cv::DMatch>
FeatureTrackingNodelet::filterMatches(const std::vector<std::vector<cv::DMatch>> &knn_matches) {
    auto matches = std::vector<cv::DMatch>{};
    for (const auto &m : knn_matches) {
        assert(m.size() >= 2);
        auto ratio = 1.0 * m[0].distance / m[1].distance;
        if (ratio < MATCH_RATIO) {
            matches.push_back(m[0]);
        }
    }
    return matches;
}

int FeatureTrackingNodelet::generateFeatureId() {
    static int id = 0;
    return id++;
}

}  // namespace feature_tracker
