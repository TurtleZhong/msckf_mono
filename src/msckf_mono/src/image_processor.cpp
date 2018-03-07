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

ImageProcessor::~ImageProcessor() {
    destroyAllWindows();
    //ROS_INFO("Feature lifetime statistics:");
    //featureLifetimeStatistics();
    return;
}


bool ImageProcessor::createRosIO()
{
//    feature_pub = nh.advertise<CameraMeasurement>(
//                "features", 3);
//    tracking_info_pub = nh.advertise<TrackingInfo>(
//                "tracking_info", 1);
//    image_transport::ImageTransport it(nh);
//    debug_stereo_pub = it.advertise("debug_stereo_image", 1);


//    cam0_img_sub.subscribe(nh, "cam0_image", 10);
//    cam1_img_sub.subscribe(nh, "cam1_image", 10);
//    stereo_sub.connectInput(cam0_img_sub, cam1_img_sub);
//    stereo_sub.registerCallback(&ImageProcessor::stereoCallback, this);
    cam0_img_sub = nh.subscribe("cam0_image", 10,
                                &ImageProcessor::monoCallback, this);
    imu_sub = nh.subscribe("imu", 50,
                           &ImageProcessor::imuCallback, this);

    return true;
}

bool ImageProcessor::initialize() {
    //  if (!loadParameters()) return false;
    //  ROS_INFO("Finish loading ROS parameters...");

    //  // Create feature detector.
    //  detector_ptr = FastFeatureDetector::create(
    //      processor_config.fast_threshold);

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
    const Mat &curr_cam0_image = cam0_curr_img_ptr->image;
    imshow("curr_image", curr_cam0_image);
    waitKey(27);
}

void ImageProcessor::imuCallback(
    const sensor_msgs::ImuConstPtr& msg) {
  // Wait for the first image to be set.
  if (is_first_img) return;
  imu_msg_buffer.push_back(*msg);
  return;
}




}


