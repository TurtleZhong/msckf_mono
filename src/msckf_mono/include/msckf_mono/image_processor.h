#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

namespace msckf_mono
{
/*
 * @brief ImageProcessor Detects and tracks features
 *    in image sequences.
 */
class ImageProcessor
{
public:
    // Constructor
    ImageProcessor(ros::NodeHandle& n);
    // Disable copy and assign constructors.
    ImageProcessor(const ImageProcessor&) = delete;
    ImageProcessor operator=(const ImageProcessor&) = delete;

    // Destructor
    ~ImageProcessor();

    // Initialize the object.
    bool initialize();

    typedef boost::shared_ptr<ImageProcessor> Ptr;
    typedef boost::shared_ptr<const ImageProcessor> ConstPtr;

private:

    /*
     * @brief ProcessorConfig Configuration parameters for
     *    feature detection and tracking.
     */
    struct ProcessorConfig {
      int grid_row;
      int grid_col;
      int grid_min_feature_num;
      int grid_max_feature_num;

      int pyramid_levels;
      int patch_size;
      int fast_threshold;
      int max_iteration;
      double track_precision;
      double ransac_threshold;
      double stereo_threshold;
    };

    /*
     * @brief FeatureIDType An alias for unsigned long long int.
     */
    typedef unsigned long long int FeatureIDType;

    /*
     * @brief FeatureMetaData Contains necessary information
     *    of a feature for easy access.
     */
    struct FeatureMetaData {
      FeatureIDType id;
      float response;
      int lifetime;
      cv::Point2f cam0_point;
//      cv::Point2f cam1_point;
    };

    /*
     * @brief GridFeatures Organize features based on the grid
     *    they belong to. Note that the key is encoded by the
     *    grid index.
     */
    typedef std::map<int, std::vector<FeatureMetaData> > GridFeatures;

    /*
     * @brief keyPointCompareByResponse
     *    Compare two keypoints based on the response.
     */
    static bool keyPointCompareByResponse(
        const cv::KeyPoint& pt1,
        const cv::KeyPoint& pt2) {
      // Keypoint with higher response will be at the
      // beginning of the vector.
      return pt1.response > pt2.response;
    }
    /*
     * @brief featureCompareByResponse
     *    Compare two features based on the response.
     */
    static bool featureCompareByResponse(
        const FeatureMetaData& f1,
        const FeatureMetaData& f2) {
      // Features with higher response will be at the
      // beginning of the vector.
      return f1.response > f2.response;
    }
    /*
     * @brief featureCompareByLifetime
     *    Compare two features based on the lifetime.
     */
    static bool featureCompareByLifetime(
        const FeatureMetaData& f1,
        const FeatureMetaData& f2) {
      // Features with longer lifetime will be at the
      // beginning of the vector.
      return f1.lifetime > f2.lifetime;
    }

    /*
     * @brief loadParameters
     *    Load parameters from the parameter server.
     */
    bool loadParameters();

    /*
     * @brief createRosIO
     *    Create ros publisher and subscirbers.
     */
    bool createRosIO();

    /*
     * @brief monoCallback
     *    Callback function for the stereo images.
     * @param cam0_img left image.
     */
    void monoCallback(
        const sensor_msgs::ImageConstPtr& cam0_img);

    /*
     * @brief imuCallback
     *    Callback function for the imu message.
     * @param msg IMU msg.
     */
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    /*
     * @brief createImagePyramids
     *    Create image pyramids used for klt tracking.
     */
    void createImagePyramids();

    /*
     * @initializeFirstFrame
     *    Initialize the image processing sequence, which is
     *    bascially detect new features on the first image.
     */
    void initializeFirstFrame();

    /*
     * @brief drawFeaturesMono
     *    Draw tracked and newly detected features on the left
     *    image only.
     */
    void drawFeaturesMono();

    // Indicate if this is the first image message.
    bool is_first_img;

    // ID for the next new feature.
    FeatureIDType next_feature_id;

    // Feature detector
    ProcessorConfig processor_config;
    cv::Ptr<cv::Feature2D> detector_ptr;

    // IMU message buffer.
    std::vector<sensor_msgs::Imu> imu_msg_buffer;

    // Camera calibration parameters
    std::string cam0_distortion_model;
    cv::Vec2i cam0_resolution;
    cv::Vec4d cam0_intrinsics;
    cv::Vec4d cam0_distortion_coeffs;

//    std::string cam1_distortion_model;
//    cv::Vec2i cam1_resolution;
//    cv::Vec4d cam1_intrinsics;
//    cv::Vec4d cam1_distortion_coeffs;

    // Take a vector from cam0 frame to the IMU frame.
    cv::Matx33d R_cam0_imu;
    cv::Vec3d t_cam0_imu;
    // Take a vector from cam1 frame to the IMU frame.
    cv::Matx33d R_cam1_imu;
    cv::Vec3d t_cam1_imu;

    // Previous and current images
    cv_bridge::CvImageConstPtr cam0_prev_img_ptr;
    cv_bridge::CvImageConstPtr cam0_curr_img_ptr;
    cv_bridge::CvImageConstPtr cam1_curr_img_ptr;

    // Pyramids for previous and current image
    std::vector<cv::Mat> prev_cam0_pyramid_;
    std::vector<cv::Mat> curr_cam0_pyramid_;
//    std::vector<cv::Mat> curr_cam1_pyramid_;

    // Features in the previous and current image.
    boost::shared_ptr<GridFeatures> prev_features_ptr;
    boost::shared_ptr<GridFeatures> curr_features_ptr;

    // Number of features after each outlier removal step.
    int before_tracking;
    int after_tracking;
    int after_matching;
    int after_ransac;

    // Ros node handle
    ros::NodeHandle nh;

    // Subscribers and publishers.
//    message_filters::Subscriber<
//      sensor_msgs::Image> cam0_img_sub;
//    message_filters::Subscriber<
//      sensor_msgs::Image> cam1_img_sub;
//    message_filters::TimeSynchronizer<
//      sensor_msgs::Image, sensor_msgs::Image> stereo_sub;
    ros::Subscriber cam0_img_sub;
    ros::Subscriber imu_sub;
    ros::Publisher feature_pub;
    ros::Publisher tracking_info_pub;
    image_transport::Publisher debug_stereo_pub;

};
typedef ImageProcessor::Ptr ImageProcessorPtr;
typedef ImageProcessor::ConstPtr ImageProcessorConstPtr;

}





#endif // IMAGE_PROCESSOR_H
