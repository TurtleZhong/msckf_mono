//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#ifndef MSCKF_EKF_HPP
#define MSCKF_EKF_HPP

#include <Eigen/Dense>

#include "msckf/types.hpp"
#include "msckf/util.hpp"

namespace msckf {

class EKF {
 public:
    EKF(const Vector3d &init_rotation, const Vector3d &init_position);
    void reset(const Vector3d &init_rotation, const Vector3d &init_position);
    MatrixXd jacobian() const;
    void estimate(double dt, const Vector3d &input_rot_vel, const Vector3d &input_vel);
    void addCameraFrame(ImageSeq image_seq);
    void addFeatures(ImageSeq image_seq, FeatureList features);

    // Update the filter state and covariance given measurement residuals r, Jacobian H, and covariance R.
    void updateEkf(const VectorXd &r, const MatrixXd &H, const MatrixXd &R);

    // Perform QR decomposition to reduce matrix size, replacing inputs
    void qrDecomposition(VectorXd &r, MatrixXd &H, MatrixXd &R);

    // Update state given change vector containing 3-angle error, setting quaternions appropriately
    void updateState(const VectorXd &state_error);

    ImuStateVector getStateEstimate() const { return state_; }

 public:
    // Parameters
    ImuErrorVector process_noise{ImuErrorVector::Zero()}; //12*1
    ImuCovMatrix process_covariance; //12*12
    double image_variance{0.1};
    bool use_qr_decomposition{false};
    int max_feature_tracks_per_update{30};

    // Accessors for parts of state and covariance
    const VectorXd& state() const {
        return state_;
    }
    const MatrixXd& covariance() const {
        return covariance_;
    }
    Eigen::Ref<VectorXd> imuState() {
        return state_.head<13>();
    }
    Eigen::Ref<VectorXd> cameraState() {
        return state_.segment(13, 7 * nCameraPoses());
    }
    Eigen::QuaternionMapd cameraQuaternion(int n) {
        return Eigen::QuaternionMapd{state_.segment<4>(13 + 7 * n).data()};
    }
    Matrix3d cameraRotation(int n) const {
        return Matrix3d{Quaterniond{state_.segment<4>(13 + 7 * n).data()}};
    }
    Eigen::Ref<Vector3d> cameraPosition(int n) {
        return state_.segment<3>(13 + 7 * n + 4);
    }
    Eigen::QuaternionMapd quaternion() {
        return Eigen::QuaternionMapd{quaternion_vector().data()};
    }
    Eigen::Map<const Quaterniond> quaternion() const {
        return Eigen::Map<const Quaterniond>{quaternion_vector().data()};
    }
    Eigen::Ref<Eigen::Vector4d> quaternion_vector() {
        return state_.head<4>();
    }
    Eigen::Ref<const Eigen::Vector4d> quaternion_vector() const {
        return Eigen::Ref<const Eigen::Vector4d>{state_.segment<4>(0)};
    }
    Eigen::Ref<Vector3d> biasGyro() {
        return state_.segment<3>(4);
    }
    Eigen::Ref<Vector3d> biasVel() {
        return state_.segment<3>(7);
    }
    Eigen::Ref<Vector3d> position() {
        return state_.segment<3>(10);
    }
    Eigen::Ref<ImuCovMatrix> imuCovariance() {
        return covariance_.topLeftCorner<12, 12>();
    }
    Eigen::Ref<MatrixXd> cameraCameraCovariance() {
        const auto N = nCameraPoses();
        return covariance_.bottomRightCorner(6 * N, 6 * N);
    }
    Eigen::Ref<MatrixXd> imuCameraCovariance() {
        const auto N = nCameraPoses();
        return covariance_.topRightCorner(12, 6 * N);
    }
    Eigen::Ref<MatrixXd> cameraImuCovariance() {
        const auto N = nCameraPoses();
        return covariance_.bottomLeftCorner(6 * N, 12);
    }

    // The number of camera poses currently in the filter state (N)
    int nCameraPoses() const { return (state_.size() - 13) / 7; }

    // The current position of the camera pose with given image sequence number
    // Throw std::range_error if not in state
    int nFromImageSeq(ImageSeq seq) const;

    InternalSeq next_camera_seq{0};
    InternalSeq last_features_seq{0};
    Quaterniond q_imu_to_camera{1, 0, 0, 0};
    Vector3d p_imu_to_camera{0, 0, 0};
    int min_track_length{5};

 private:
    int nFromInternalSeq(InternalSeq seq) const;
    InternalSeq addImageSeq(ImageSeq image_seq);
    void processFeatures();

    // Extract features which have moved out of the frame.
    // Return them and erase them from the features buffer.
    std::vector<FeatureInstanceList> filterFeatures();

    VectorXd state_{13};
    MatrixXd covariance_{12, 12};

    // map of features for frames currently in the state
    std::map<FeatureId, FeatureInstanceList> features_;

    // features for frames for which are not part of the state yet
    std::map<ImageSeq, FeatureList> pending_features_;

    // map of given image sequence number to internal sequence number
    std::map<ImageSeq, InternalSeq> image_seqs;
    bool isFeatureExpired(const FeatureInstanceList &instances) const;
    bool isFeatureUsable(const FeatureInstanceList &instances) const;

    // Given feature measurements, estimate positions, and output residuals and Jacobian.
    // The residuals and Jacobian are already uncorrelated to the state, but not marginalized.
    void estimateFeaturePositions(const std::vector<FeatureInstanceList> &features,
                                  VectorXd &r_o,
                                  MatrixXd &H_o,
                                  MatrixXd &R_o);

    // Calculate measurement Jacobian H for a single feature track
    void singleFeatureH(const Vector3d &estimated_global_pos,
                        const std::vector<int> &cameraIndices,
                        MatrixXd &H_X_j,
                        MatrixXd &H_f_j);

    // Project the residuals, r, of a single feature track onto the left nullspace
    // of the matrix H as described in Mourikis. Work in place.
    void projectLeftNullspace(const MatrixXd &H_f_j, VectorXd &r, MatrixXd &H_X_j);

};

const int COV_ROT = 0;
const int COV_BG = 3;
const int COV_BV = 6;
const int COV_POS = 9;

}  // namespace msckf

#endif  // MSCKF_EKF_HPP
