//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//re


#include "msckf/msckf_nodelet.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

PLUGINLIB_EXPORT_CLASS(msckf::MsckfNodelet,
                       nodelet::Nodelet);

namespace msckf {

MsckfNodelet::MsckfNodelet() {
    ekf.process_noise.segment(COV_POS, 3) = 0.01 * Vector3d::Ones();
    ekf.process_noise.segment(COV_BG, 3) = 0.0001 * Vector3d::Ones();
    ekf.process_noise.segment(COV_BV, 3) = 0.0001 * Vector3d::Ones();
    ekf.process_noise.segment(COV_ROT, 3) = 0.01 * Vector3d::Ones();


    // Set camera calibration for KITTI
    // todo: move elsewhere
    Eigen::Matrix3d R_velo_to_cam, R_imu_to_velo, R_rect;
    R_imu_to_velo << 9.999976e-01, 7.553071e-04, -2.035826e-03,
            -7.854027e-04, 9.998898e-01, -1.482298e-02,
            2.024406e-03, 1.482454e-02, 9.998881e-01;
    R_velo_to_cam << 7.533745e-03, -9.999714e-01, -6.166020e-04,
            1.480249e-02, 7.280733e-04, -9.998902e-01,
            9.998621e-01, 7.523790e-03, 1.480755e-02;
    R_rect << 9.999239e-01, 9.837760e-03, -7.445048e-03,
            -9.869795e-03, 9.999421e-01, -4.278459e-03,
            7.402527e-03, 4.351614e-03, 9.999631e-01;
    Eigen::Vector3d t_imu_to_velo{-8.086759e-01, 3.195559e-01, -7.997231e-01};
    Eigen::Vector3d t_velo_to_cam{-4.069766e-03, -7.631618e-02, -2.717806e-01};

    ekf.q_imu_to_camera = Eigen::Quaterniond{R_rect * R_velo_to_cam * R_imu_to_velo};
    ekf.p_imu_to_camera = t_velo_to_cam + t_imu_to_velo;

    initLogging();
}

MsckfNodelet::~MsckfNodelet() {

}

void MsckfNodelet::onInit() {
    auto nh = getNodeHandle();
    auto image_transport = image_transport::ImageTransport{getNodeHandle()};

    odom_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("odom_combined", 10);

    imu_sub = nh.subscribe("/imu_vel", 10, &MsckfNodelet::imuCallback, this);
    odom_sub = nh.subscribe("/odom", 10, &MsckfNodelet::odomCallback, this);
    image_sub = image_transport.subscribe("/camera/image_rect", 1,
                                          &MsckfNodelet::imageCallback, this);
    features_sub = nh.subscribe("/features", 10, &MsckfNodelet::featuresCallback, this);

}

void MsckfNodelet::publishStateEstimate() {
    auto msg = geometry_msgs::PoseWithCovarianceStamped{};

    // todo: make conversion functions
    auto pos = geometry_msgs::Point{};
    pos.x = ekf.position()(0);
    pos.y = ekf.position()(1);
    pos.z = ekf.position()(2);
    auto q = geometry_msgs::Quaternion{};
    q.x = ekf.quaternion().x();
    q.y = ekf.quaternion().y();
    q.z = ekf.quaternion().z();
    q.w = ekf.quaternion().w();

    // covariance: row-major 6x6 array in order (x, y, z, rot_x, rot_y, rot_z)
    Eigen::Matrix<double, 6, 6> cov;
    cov.block(0, 0, 3, 3) = ekf.imuCovariance().block<3, 3>(COV_POS, COV_POS);
    cov.block(0, 3, 3, 3) = ekf.imuCovariance().block<3, 3>(COV_POS, COV_ROT);
    cov.block(3, 0, 3, 3) = ekf.imuCovariance().block<3, 3>(COV_ROT, COV_POS);
    cov.block(3, 3, 3, 3) = ekf.imuCovariance().block<3, 3>(COV_ROT, COV_ROT);

    Eigen::Map<Eigen::Matrix<double, 6, 6>>{msg.pose.covariance.data(), 6, 6} = cov;

    msg.pose.pose.position = pos;
    msg.pose.pose.orientation = q;
    odom_pub.publish(msg);
}

void MsckfNodelet::imuCallback(const geometry_msgs::TwistStampedConstPtr &msg) {
    // Use message timestamps to calculate dt.
    // The first dt will be zero.
    static auto last_msg_stamp = msg->header.stamp;
    auto diff = (msg->header.stamp - last_msg_stamp);
    auto dt = (msg->header.stamp - last_msg_stamp).toSec();

    const auto rot_vel = -msckf::Vector3d{msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
    const auto vel = msckf::Vector3d{msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};

    ekf.estimate(dt, rot_vel, vel);

    last_msg_stamp = msg->header.stamp;

    publishStateEstimate();
}

void MsckfNodelet::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    // Reset the EKF after we have an initial position and initial orientation
    static auto messages_received = 0;
    auto &pos = msg->pose.pose.position;
    const auto position = msckf::Vector3d{pos.x, pos.y, pos.z};
    static auto prev_position = position;
    if (messages_received == 0) {
        messages_received = 1;
        // Done. Wait for next message
    } else if ((messages_received == 1)) {
        // Calculate orientation
        const msckf::Vector3d d1{1, 0, 0};
        const msckf::Vector3d d2{(position - prev_position).normalized()};
        const msckf::Vector3d orientation = xyzFromQuaternion(msckf::Quaterniond{}.setFromTwoVectors(d2, d1));

        // Reset EKF with initial position
        ekf.reset(orientation, position);
        NODELET_INFO_STREAM("Initialized EKF with position (" << position.transpose()
                                                              << "), orientation ("
                                                              << 180.0 / M_PI * orientation.transpose() << ") deg");
        messages_received = 2;
    }
}

void MsckfNodelet::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    /* Add a camera pose estimate to the EKF state
     *
     * We separately get extracted features in featuresCallback(), but they are tagged with the same
     * image_seq number. we let the EKF class store this.
     */

    ekf.addCameraFrame(msg->header.seq);

    // todo: check if window length exceeded
}

void MsckfNodelet::featuresCallback(const msckf::ImageFeaturesConstPtr &msg) {
    auto features = FeatureList{};
    for (const auto &f : msg->features) {
        // Note feature positions should already be idealized
        features.push_back(ImageFeature{f.id, {f.position.x, f.position.y}});
    }
    ekf.addFeatures(msg->image_seq, features);

    NODELET_INFO_STREAM("IMU state:"
                                << "\n    rot deg" << xyzFromQuaternion(ekf.quaternion()).transpose() * 180 / M_PI
                                << "\n    pos    " << ekf.position().transpose()
                                << "\n    bias_g " << ekf.biasGyro().transpose() * 180 / M_PI
                                << "\n    bias_v " << ekf.biasVel().transpose());
}
void MsckfNodelet::initLogging() {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::debug);
}

}  // namespace msckf

