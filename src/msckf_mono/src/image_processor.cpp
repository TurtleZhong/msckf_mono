#include <iostream>
#include <algorithm>
#include <set>
#include <Eigen/Dense>

#include <sensor_msgs/image_encodings.h>
#include <random_numbers/random_numbers.h>

#include <msckf_mono/image_processor.h>
#include <msckf_mono/utils.h>

using namespace std;
using namespace cv;
using namespace Eigen;

namespace msckf_mono
{
ImageProcessor::ImageProcessor(ros::NodeHandle& n) :
    nh(n),
    is_first_img(true),
    //img_transport(n),
    prev_features_ptr(new GridFeatures()),
    curr_features_ptr(new GridFeatures()) {
    return;
}

ImageProcessor::~ImageProcessor()
{
    destroyAllWindows();
    //ROS_INFO("Feature lifetime statistics:");
    //featureLifetimeStatistics();
    return;
}

bool ImageProcessor::loadParameters()
{
    // Camera calibration parameters
    nh.param<string>("cam0/distortion_model",
                     cam0_distortion_model, string("radtan"));

    vector<int> cam0_resolution_temp(2);
    nh.getParam("cam0/resolution", cam0_resolution_temp);
    cam0_resolution[0] = cam0_resolution_temp[0];
    cam0_resolution[1] = cam0_resolution_temp[1];

    vector<double> cam0_intrinsics_temp(4);
    nh.getParam("cam0/intrinsics", cam0_intrinsics_temp);
    cam0_intrinsics[0] = cam0_intrinsics_temp[0];
    cam0_intrinsics[1] = cam0_intrinsics_temp[1];
    cam0_intrinsics[2] = cam0_intrinsics_temp[2];
    cam0_intrinsics[3] = cam0_intrinsics_temp[3];

    vector<double> cam0_distortion_coeffs_temp(4);
    nh.getParam("cam0/distortion_coeffs",
                cam0_distortion_coeffs_temp);
    cam0_distortion_coeffs[0] = cam0_distortion_coeffs_temp[0];
    cam0_distortion_coeffs[1] = cam0_distortion_coeffs_temp[1];
    cam0_distortion_coeffs[2] = cam0_distortion_coeffs_temp[2];
    cam0_distortion_coeffs[3] = cam0_distortion_coeffs_temp[3];

    cv::Mat     T_imu_cam0 = utils::getTransformCV(nh, "cam0/T_cam_imu");
    cv::Matx33d R_imu_cam0(T_imu_cam0(cv::Rect(0,0,3,3)));
    cv::Vec3d   t_imu_cam0 = T_imu_cam0(cv::Rect(3,0,1,3));
    R_cam0_imu = R_imu_cam0.t();
    t_cam0_imu = -R_imu_cam0.t() * t_imu_cam0;

    cv::Mat T_cam0_cam1 = utils::getTransformCV(nh, "cam1/T_cn_cnm1");
    cv::Mat T_imu_cam1 = T_cam0_cam1 * T_imu_cam0;
    cv::Matx33d R_imu_cam1(T_imu_cam1(cv::Rect(0,0,3,3)));
    cv::Vec3d   t_imu_cam1 = T_imu_cam1(cv::Rect(3,0,1,3));
    R_cam1_imu = R_imu_cam1.t();
    t_cam1_imu = -R_imu_cam1.t() * t_imu_cam1;

    // Processor parameters
    nh.param<int>("grid_row", processor_config.grid_row, 4);
    nh.param<int>("grid_col", processor_config.grid_col, 4);
    nh.param<int>("grid_min_feature_num",
                  processor_config.grid_min_feature_num, 2);
    nh.param<int>("grid_max_feature_num",
                  processor_config.grid_max_feature_num, 4);
    nh.param<int>("pyramid_levels",
                  processor_config.pyramid_levels, 3);
    nh.param<int>("patch_size",
                  processor_config.patch_size, 31);
    nh.param<int>("fast_threshold",
                  processor_config.fast_threshold, 20);
    nh.param<int>("max_iteration",
                  processor_config.max_iteration, 30);
    nh.param<double>("track_precision",
                     processor_config.track_precision, 0.01);
    nh.param<double>("ransac_threshold",
                     processor_config.ransac_threshold, 3);
    nh.param<double>("stereo_threshold",
                     processor_config.stereo_threshold, 3);

    ROS_INFO("===========================================");
    ROS_INFO("cam0_resolution: %d, %d",
             cam0_resolution[0], cam0_resolution[1]);
    ROS_INFO("cam0_intrinscs: %f, %f, %f, %f",
             cam0_intrinsics[0], cam0_intrinsics[1],
            cam0_intrinsics[2], cam0_intrinsics[3]);
    ROS_INFO("cam0_distortion_model: %s",
             cam0_distortion_model.c_str());
    ROS_INFO("cam0_distortion_coefficients: %f, %f, %f, %f",
             cam0_distortion_coeffs[0], cam0_distortion_coeffs[1],
            cam0_distortion_coeffs[2], cam0_distortion_coeffs[3]);

    cout << R_imu_cam0 << endl;
    cout << t_imu_cam0.t() << endl;

    ROS_INFO("grid_row: %d",
             processor_config.grid_row);
    ROS_INFO("grid_col: %d",
             processor_config.grid_col);
    ROS_INFO("grid_min_feature_num: %d",
             processor_config.grid_min_feature_num);
    ROS_INFO("grid_max_feature_num: %d",
             processor_config.grid_max_feature_num);
    ROS_INFO("pyramid_levels: %d",
             processor_config.pyramid_levels);
    ROS_INFO("patch_size: %d",
             processor_config.patch_size);
    ROS_INFO("fast_threshold: %d",
             processor_config.fast_threshold);
    ROS_INFO("max_iteration: %d",
             processor_config.max_iteration);
    ROS_INFO("track_precision: %f",
             processor_config.track_precision);
    ROS_INFO("ransac_threshold: %f",
             processor_config.ransac_threshold);
    ROS_INFO("stereo_threshold: %f",
             processor_config.stereo_threshold);
    ROS_INFO("===========================================");
    return true;
}

bool ImageProcessor::createRosIO()
{
    //    feature_pub = nh.advertise<CameraMeasurement>(
    //                "features", 3);
    //    tracking_info_pub = nh.advertise<TrackingInfo>(
    //                "tracking_info", 1);
    //    image_transport::ImageTransport it(nh);
    //    debug_stereo_pub = it.advertise("debug_stereo_image", 1);

    cam0_img_sub = nh.subscribe("cam0_image", 10,
                                &ImageProcessor::monoCallback, this);
    imu_sub = nh.subscribe("imu", 50,
                           &ImageProcessor::imuCallback, this);

    return true;
}

bool ImageProcessor::initialize() {
    if (!loadParameters()) return false;
    ROS_INFO("Finish loading ROS parameters...");

    // Create feature detector.
    detector_ptr = FastFeatureDetector::create(
                processor_config.fast_threshold); //10

    if (!createRosIO()) return false;
    ROS_INFO("Finish creating ROS IO...");

    return true;
}


void ImageProcessor::monoCallback(
        const sensor_msgs::ImageConstPtr &cam0_img)
{
    // Get the current image.
    cam0_curr_img_ptr = cv_bridge::toCvShare(cam0_img,
                                             sensor_msgs::image_encodings::MONO8);
    //    const Mat &curr_cam0_image = cam0_curr_img_ptr->image;
    //    imshow("curr_image", curr_cam0_image);
    //    waitKey(27);

    // Build the image pyramids once since they're used at multiple places
    createImagePyramids();

    if(is_first_img)
    {
        // Detect features in the first frame
        initializeFirstFrame();
        is_first_img = false;
    }
    else
    {
        // Track the feature in the previous image.
        ROS_INFO("Need to track the features");
        drawFeaturesMono();
    }

    // Update the previous image and previous features.
    cam0_prev_img_ptr = cam0_curr_img_ptr;
    prev_features_ptr = curr_features_ptr;
    std::swap(prev_cam0_pyramid_, curr_cam0_pyramid_);

    // Initialize the current features to empty vectors.
    curr_features_ptr.reset(new GridFeatures());
    for (int code = 0; code <
        processor_config.grid_row*processor_config.grid_col; ++code) {
      (*curr_features_ptr)[code] = vector<FeatureMetaData>(0);
    }

}

void ImageProcessor::imuCallback(
        const sensor_msgs::ImuConstPtr& msg)
{
    // Wait for the first image to be set.
    if (is_first_img) return;
    imu_msg_buffer.push_back(*msg);
    return;
}

void ImageProcessor::createImagePyramids()
{
    const Mat& curr_cam0_img = cam0_curr_img_ptr->image;
    buildOpticalFlowPyramid(
                curr_cam0_img, curr_cam0_pyramid_,
                Size(processor_config.patch_size, processor_config.patch_size),
                processor_config.pyramid_levels, true, BORDER_REFLECT_101,
                BORDER_CONSTANT, false);
}

void ImageProcessor::initializeFirstFrame()
{
    // Size of each grid.
    const Mat& img = cam0_curr_img_ptr->image;
    static int grid_height = img.rows / processor_config.grid_row;
    static int grid_width = img.cols / processor_config.grid_col;

    // Detect new features on the frist image.
    vector<KeyPoint> new_features(0);
    detector_ptr->detect(img, new_features); //FastFeatureDetector

    Mat out_img = img.clone();
    cvtColor(out_img, out_img, CV_GRAY2BGR);
    //    drawKeypoints(img, new_features, out_img);
    //    imshow("first_frame", out_img);
    //    ROS_INFO("Feature.size: %d",
    //             new_features.size());
    //    waitKey(0);

    // Find the stereo matched points for the newly
    // detected features.
    vector<cv::Point2f> cam0_points(new_features.size());
    for (int i = 0; i < new_features.size(); ++i)
        cam0_points[i] = new_features[i].pt;

    vector<unsigned char> inlier_markers(cam0_points.size(), 1);
    //    stereoMatch(cam0_points, cam1_points, inlier_markers);


    vector<cv::Point2f> cam0_inliers(0);
    vector<float> response_inliers(0);
    for (int i = 0; i < inlier_markers.size(); ++i)
    {
        if (inlier_markers[i] == 0) continue;
        cam0_inliers.push_back(cam0_points[i]);
        response_inliers.push_back(new_features[i].response);
    }

    // Group the features into grids
    GridFeatures grid_new_features;
    for (int code = 0; code <
         processor_config.grid_row*processor_config.grid_col; ++code)
        grid_new_features[code] = vector<FeatureMetaData>(0);

    for (int i = 0; i < cam0_inliers.size(); ++i) {
        const cv::Point2f& cam0_point = cam0_inliers[i];
        const float& response = response_inliers[i];

        int row = static_cast<int>(cam0_point.y / grid_height);
        int col = static_cast<int>(cam0_point.x / grid_width);
        int code = row*processor_config.grid_col + col;

        FeatureMetaData new_feature;
        new_feature.response = response;
        new_feature.cam0_point = cam0_point;
        grid_new_features[code].push_back(new_feature);
    }

    // Sort the new features in each grid based on its response.
    for (auto& item : grid_new_features)
        std::sort(item.second.begin(), item.second.end(),
                  &ImageProcessor::featureCompareByResponse);

    // Collect new features within each grid with high response.
    for (int code = 0; code <
         processor_config.grid_row*processor_config.grid_col; ++code) {
        vector<FeatureMetaData>& features_this_grid = (*curr_features_ptr)[code];
        vector<FeatureMetaData>& new_features_this_grid = grid_new_features[code];

        for (int k = 0; k < processor_config.grid_min_feature_num &&
             k < new_features_this_grid.size(); ++k) {
            features_this_grid.push_back(new_features_this_grid[k]);
            features_this_grid.back().id = next_feature_id++;
            features_this_grid.back().lifetime = 1;
        }
    }

    for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code)
    {
        vector<FeatureMetaData> &features_this_grid = (*curr_features_ptr)[code];
        for(int i = 0; i < features_this_grid.size(); i++)
        {
            circle(out_img, features_this_grid[i].cam0_point,
                   3, Scalar(0, 255, 0), -1);
        }
    }

    imshow("features", out_img);
    waitKey(27);

    return;
}


void ImageProcessor::drawFeaturesMono()
{
    // Colors for different features.
    Scalar tracked(0, 255, 0);
    Scalar new_feature(0, 255, 255);

    static int grid_height =
            cam0_curr_img_ptr->image.rows / processor_config.grid_row;
    static int grid_width =
            cam0_curr_img_ptr->image.cols / processor_config.grid_col;

    // Create an output image.
    int img_height = cam0_curr_img_ptr->image.rows;
    int img_width = cam0_curr_img_ptr->image.cols;
    Mat out_img(img_height, img_width, CV_8UC3);
    cvtColor(cam0_curr_img_ptr->image, out_img, CV_GRAY2RGB);

    // Draw grids on the image.
    for (int i = 1; i < processor_config.grid_row; ++i) {
        Point pt1(0, i*grid_height);
        Point pt2(img_width, i*grid_height);
        line(out_img, pt1, pt2, Scalar(255, 0, 0));
    }
    for (int i = 1; i < processor_config.grid_col; ++i) {
        Point pt1(i*grid_width, 0);
        Point pt2(i*grid_width, img_height);
        line(out_img, pt1, pt2, Scalar(255, 0, 0));
    }

    // Collect features ids in the previous frame.
    vector<FeatureIDType> prev_ids(0);
    for (const auto& grid_features : *prev_features_ptr)
        for (const auto& feature : grid_features.second)
            prev_ids.push_back(feature.id);

    // Collect feature points in the previous frame.
    map<FeatureIDType, Point2f> prev_points;
    for (const auto& grid_features : *prev_features_ptr)
        for (const auto& feature : grid_features.second)
            prev_points[feature.id] = feature.cam0_point;

    // Collect feature points in the current frame.
    map<FeatureIDType, Point2f> curr_points;
    for (const auto& grid_features : *curr_features_ptr)
        for (const auto& feature : grid_features.second)
            curr_points[feature.id] = feature.cam0_point;

    // Draw tracked features.
    for (const auto& id : prev_ids) {
        if (prev_points.find(id) != prev_points.end() &&
                curr_points.find(id) != curr_points.end()) {
            cv::Point2f prev_pt = prev_points[id];
            cv::Point2f curr_pt = curr_points[id];
            circle(out_img, curr_pt, 3, tracked);
            line(out_img, prev_pt, curr_pt, tracked, 1);

            prev_points.erase(id);
            curr_points.erase(id);
        }
    }

    // Draw new features.
    for (const auto& new_curr_point : curr_points) {
        cv::Point2f pt = new_curr_point.second;
        circle(out_img, pt, 3, new_feature, -1);
    }

    imshow("Feature", out_img);
    waitKey(0);
}




}


