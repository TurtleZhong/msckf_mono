//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#include "msckf/EKF.hpp"

#include <boost/log/trivial.hpp>

#include "msckf/estimation.hpp"

namespace msckf {

static const auto GRAVITY_ACCEL = Vector3d{0, 0, -9.81};

EKF::EKF(const Vector3d &init_rotation, const Vector3d &init_position) {
    reset(init_rotation, init_position);
}

void EKF::reset(const Vector3d &init_rotation, const Vector3d &init_position) {
    BOOST_LOG_TRIVIAL(info) << "Resetting filter state and removing all camera frames.";

    state_.resize(13);
    state_ << quaternionFromXYZ(init_rotation).coeffs(), Vector3d::Zero(), Vector3d::Zero(), init_position;
    process_covariance = process_noise.cwiseProduct(process_noise).asDiagonal();
    covariance_.resize(12, 12);
    covariance_ = 0.0001 * ImuErrorVector::Ones().asDiagonal();

    features_.clear();
    pending_features_.clear();
    image_seqs.clear();
}

MatrixXd EKF::jacobian() const {
    MatrixXd J = MatrixXd::Zero(6, 12 + 6 * nCameraPoses());
    J.block<3, 3>(0, 0) = q_imu_to_camera.matrix();
    J.block<3, 3>(3, 0) = crossMatrix(quaternion().matrix().transpose() * p_imu_to_camera);
    J.block<3, 3>(3, 9) = Matrix3d::Identity();
    return J;
}

void EKF::estimate(double dt, const Vector3d &input_rot_vel, const Vector3d &input_vel) {
    // Update the state estimate using the nonlinear motion model

    // Subtract bias from input
    Vector3d vel = input_vel - biasVel();
    Vector3d rot_vel = input_rot_vel - biasGyro();

    // construct Omega rotation matrix
    Eigen::Matrix4d big_omega;
    big_omega.block(0, 0, 3, 3) = -crossMatrix(rot_vel);
    big_omega.block(0, 3, 3, 1) = rot_vel;
    big_omega.block(3, 0, 1, 3) = -rot_vel.transpose();
    big_omega(3, 3) = 0;

    Eigen::Vector4d quaternion_dt = 0.5 * big_omega * this->quaternion_vector();

    Matrix3d C = this->quaternion().matrix();
    Matrix3d CT = C.transpose();
    Vector3d position_dt = CT * vel;

    // Update state
    this->quaternion_vector() += dt * quaternion_dt;
    this->quaternion().normalize();
    this->position() += dt * position_dt;

    // Update covariance

    // Jacobian
    ImuCovMatrix F = ImuCovMatrix::Zero();
    // Remember the order is rotation, bias_gyro, bias_velocity, position
    F.block(COV_ROT, COV_ROT, 3, 3) = -crossMatrix(rot_vel);      // rotation wrt rotation
    F.block(COV_ROT, COV_BG, 3, 3) = -Matrix3d::Identity();  // rotation wrt gyro bias
    F.block(COV_POS, COV_ROT, 3, 3) = -CT * crossMatrix(vel);  // velocity wrt rotation
    F.block(COV_POS, COV_BV, 3, 3) = -CT;

    // G (process noise multiplier)
    ImuCovMatrix G = ImuCovMatrix::Zero();
    G.block(COV_ROT, COV_ROT, 3, 3) = -Matrix3d::Identity();
    G.block(COV_BG, COV_BG, 3, 3) = Matrix3d::Identity();
    G.block(COV_BV, COV_BV, 3, 3) = Matrix3d::Identity();
    G.block(COV_BV, COV_POS, 3, 3) = -CT;

    // State transition matrix
    ImuCovMatrix Phi = ImuCovMatrix::Identity() + (dt * F);

    // Update covariance in blocks
    this->imuCovariance() = Phi * this->imuCovariance() * Phi.transpose()
            + G * this->process_covariance * G.transpose() * dt;
    this->imuCameraCovariance() = Phi * this->imuCameraCovariance();
    this->cameraImuCovariance() = this->cameraImuCovariance() * Phi.transpose();


    // Make positive semidefinite
    this->covariance_.diagonal() = this->covariance_.diagonal().cwiseAbs();
    this->covariance_ = 0.5 * this->covariance_ + 0.5 * this->covariance_.transpose().eval();
}

void EKF::addCameraFrame(ImageSeq image_seq) {
    // Calculate camera pose estimate, from IMU pose estimate
    Quaterniond q_camera_estimate = q_imu_to_camera * quaternion();
    Vector3d p_camera_estimate = position() + quaternion().matrix().transpose() * p_imu_to_camera;

    const auto N = nCameraPoses();  // number of camera poses already stored
    const MatrixXd J = jacobian(); // calculate jacobian before resizing, so it uses previous N

    // Augment state
    state_.conservativeResize(state_.size() + 7);

    cameraQuaternion(N) = q_camera_estimate;
    cameraPosition(N) = p_camera_estimate;

    // Augment covariance matrix
    MatrixXd ext{18 + 6 * N, 12 + 6 * N};
    ext << MatrixXd::Identity(12 + 6 * N, 12 + 6 * N), J;

    covariance_ = (ext * covariance_ * ext.transpose()).eval();

    // Add sequence number mapping
    const auto internal_seq = addImageSeq(image_seq);

    BOOST_LOG_TRIVIAL(debug) << "Added image " << image_seq << " as frame " << internal_seq;

    // Check if we already got the features for this image
    if (pending_features_.count(image_seq)) {
        addFeatures(image_seq, pending_features_[image_seq]);
        pending_features_.erase(image_seq);
    }
}

int EKF::nFromInternalSeq(InternalSeq seq) const {
    const auto n = seq - next_camera_seq + nCameraPoses();
    if (n < 0 || seq >= next_camera_seq) {
        throw std::out_of_range("That camera pose is not in the filter state");
    }
    return n;
}

InternalSeq EKF::addImageSeq(ImageSeq image_seq) {
    image_seqs.insert({image_seq, next_camera_seq});
    return next_camera_seq++;
}

int EKF::nFromImageSeq(ImageSeq seq) const {
    return nFromInternalSeq(image_seqs.at(seq));
}

void EKF::addFeatures(ImageSeq image_seq, FeatureList features) {
    BOOST_LOG_TRIVIAL(debug) << "addFeatures(): " << features.size()
                             << " instances in image " << image_seq;

    // Did we already have addCameraFrame() called with this image_seq?
    if (image_seqs.count(image_seq)) {
        for (auto &f : features) {
            last_features_seq = image_seqs[image_seq];
            features_[f.id].push_back(FeatureInstance{last_features_seq, f.point});
        }
        processFeatures();
    } else {
        // We got the features before the frame (it's possible). Keep for later.
        pending_features_[image_seq] = features;
    }
}
void EKF::processFeatures() {
    // Put together a list of features to use in calculations
    const auto features_to_use = filterFeatures();

    BOOST_LOG_TRIVIAL(info) << "Processing " << features_to_use.size() << " feature tracks";

    if (features_to_use.empty()) {
        return;
    }

    // Get stacked, uncorrelated residuals and Jacobian
    auto r_o = VectorXd{};  // residuals
    auto H_o = MatrixXd{};  // measurement Jacobian
    auto R_o = MatrixXd{};  // measurement covariance
    estimateFeaturePositions(features_to_use, r_o, H_o, R_o);

    BOOST_LOG_TRIVIAL(debug) << "Updating EKF with residual size " << r_o.size()
                             << ", Jacobian size " << H_o.rows() << "x" << H_o.cols();

    if (use_qr_decomposition) {
        qrDecomposition(r_o, H_o, R_o);
        BOOST_LOG_TRIVIAL(debug) << "After QR, Jacobian is " << H_o.rows() << "x" << H_o.cols();
    }

    // Update the filter
    updateEkf(r_o, H_o, R_o);
}

std::vector<FeatureInstanceList> EKF::filterFeatures() {
    auto features_to_use = std::vector<FeatureInstanceList>{};

    for (auto it = features_.cbegin(); it != features_.cend();) {
        const auto &instances = it->second;
        if (isFeatureExpired(instances)) {
            // This feature does not exist in the latest frame, therefore it has moved out of the frame.
            if (isFeatureUsable(instances) && features_to_use.size() < max_feature_tracks_per_update) {
                features_to_use.push_back(instances);
            }
            // Erase the feature from the list
            it = features_.erase(it);
        } else {
            ++it;
        }
    }
    return features_to_use;
}

bool EKF::isFeatureExpired(const FeatureInstanceList &instances) const {
    return (instances.back().seq < last_features_seq);
}
bool EKF::isFeatureUsable(const FeatureInstanceList &instances) const {
    return (instances.size() > min_track_length);
}
void EKF::estimateFeaturePositions(const std::vector<FeatureInstanceList> &features,
                                   VectorXd &r_o,
                                   MatrixXd &H_o,
                                   MatrixXd &R_o) {
    /* Given a list of feature measurements (2d ideal pixel positions) and corresponding
     * camera pose estimate, we estimate 3d feature position by minimizing residuals.
     */

    // Prepare stacked residual and Jacobian matrices. First we need the total number of feature tracks
    const auto total_rows = sizeOfNestedContainers(features);
    const auto N = nCameraPoses();
    const auto L = features.size();
    const auto n_residuals = 2 * total_rows - 3 * L;
    r_o.resize(n_residuals);  // residuals
    H_o.resize(n_residuals, 12 + 6 * N);  // measurement Jacobian

    auto total_i = 0;
    for (const auto &f : features) {
        // Collect the feature measurements and corresponding camera pose estimates
        // todo: seems inefficient
        const auto M = f.size();

        auto measurements = VectorOfVector2d{};
        auto camera_rotations = VectorOfMatrix3d{};
        auto camera_positions = VectorOfVector3d{};
        auto camera_indices = std::vector<int>{};

        for (const auto &instance : f) {
            measurements.push_back(instance.point);
            const auto n = nFromInternalSeq(instance.seq);
            camera_rotations.emplace_back(cameraQuaternion(n));
            camera_positions.emplace_back(cameraPosition(n));
            camera_indices.push_back(n);
        }

        auto residuals = VectorXd{2 * M};
        auto estimated_pos = estimateFeaturePosition(measurements, camera_rotations, camera_positions, residuals);
        auto estimated_local = Vector3d{camera_rotations.front() * (estimated_pos - camera_positions.front())};

        // Calculate Jacobians for this feature estimate
        auto H_X_j = MatrixXd{2 * M, 12 + 6 * N};
        auto H_f_j = MatrixXd{2 * M, 3};
        singleFeatureH(estimated_pos, camera_indices, H_X_j, H_f_j);

        // Get uncorrelated residuals
        projectLeftNullspace(H_f_j, residuals, H_X_j);

        // After that we expect the size to be smaller:
        const auto new_dim = 2 * M - 3;
        assert(new_dim == residuals.size());
        assert(new_dim == H_X_j.rows());

        // Push onto stacked matrices (todo: simplify)
        r_o.segment(total_i, new_dim) << residuals;
        H_o.block(total_i, 0, new_dim, 12 + 6 * N) << H_X_j;

        total_i += new_dim;
    }

    assert(total_i == n_residuals);

    // Set covariance, which is assumed to be only diagonal pixel error
    R_o.resize(n_residuals, n_residuals);
    R_o = image_variance * MatrixXd::Identity(n_residuals, n_residuals);
}

void EKF::projectLeftNullspace(const MatrixXd &H_f_j, VectorXd &r, MatrixXd &H_X_j) {
    // As in Mourikis, A is matrix whose columns form basic of left nullspace of H_f.
    // Note left nullspace == kernel of transpose
    MatrixXd A = H_f_j.transpose().fullPivLu().kernel();
    H_X_j = (A.transpose() * H_X_j).eval();
    r = (A.transpose() * r).eval();
}

void EKF::singleFeatureH(const Vector3d &estimated_global_pos,
                         const std::vector<int> &cameraIndices,
                         MatrixXd &H_X_j,
                         MatrixXd &H_f_j) {
    const auto M = cameraIndices.size();
    const auto N = nCameraPoses();

    H_X_j.setZero(2 * M, 12 + 6 * N);
    H_f_j.resize(2 * M, 3);

    for (auto i = 0 * M; i < M; ++i) {
        const auto &n = cameraIndices[i];
        const auto &C = cameraRotation(n);  // camera rotation estimate in global frame
        const auto &p = cameraPosition(n);  // camera position estimate in global frame
        const auto local_pos = Vector3d{C * (estimated_global_pos - p)};  // feature position estimate in camera frame

        // Fill in Jacobian block as given in Mourikis preprint (referenced in paper)
        auto J_i = Eigen::Matrix<double, 2, 3>{};
        J_i << 1, 0, -local_pos(0) / local_pos(2), 0, 1, -local_pos(1) / local_pos(2);
        J_i = J_i / local_pos(2);

        H_X_j.block<2, 6>(2 * i, 12 + 6 * n) << J_i * crossMatrix(local_pos), -J_i * C;

        H_f_j.block<2, 3>(2 * i, 0) << J_i * C;
    }
}

void EKF::updateEkf(const VectorXd &r, const MatrixXd &H, const MatrixXd &R) {
    const auto l = r.size();
    const auto N = nCameraPoses();
    const auto d = 12 + 6 * N;
    assert(use_qr_decomposition || l == H.rows());
    assert(d == H.cols());
    assert(l == R.rows());
    assert(l == R.cols());

    // Calculate Kalman gain
    const auto K = MatrixXd{covariance_ * H.transpose() * (H * covariance_ * H.transpose() + R).inverse()};
    const auto state_change = VectorXd{K * r};

//    BOOST_LOG_TRIVIAL(trace) << "state_change = " << state_change.transpose();

    // Note that state_change has rotations as 3 angles, while state has quaternions.
    // Thus we can't just add it to the state.
    assert(state_change.size() == state_.size() - 1 - N);
    assert(d == K.rows());
    assert(l == K.cols());

    // Update state estimate
    updateState(state_change);

    // Update covariance estimate
    covariance_ = (MatrixXd::Identity(d, d) - K * H) * covariance_ * (MatrixXd::Identity(d, d) - K * H).transpose()
            + K * R * K.transpose();

    BOOST_LOG_TRIVIAL(trace) << "new state:"
                             << "\npos " << position().format(CommaFormat)
                             << "\ndeg " << (xyzFromQuaternion(quaternion()) * 180 / M_PI).format(CommaFormat)
                             << "\nbias gyro " << (biasGyro() * 180 / M_PI).format(CommaFormat)
                             << "\nbias vel  " << biasVel().format(CommaFormat)
                             << "\n";
}

void EKF::updateState(const VectorXd &state_error) {
    // The state holds quaternions but state_error should hold angle errors.
    const auto N = nCameraPoses();
    assert(state_error.size() == state_.size() - 1 - N);
    assert(state_error.size() == 12 + 6 * N);

    // Update quaternion multiplicatively
    auto dq = errorQuaternion(state_error.segment<3>(0));
    quaternion() = dq * quaternion();

    // Update rest of IMU state additively
    state_.segment<9>(4) += state_error.segment<9>(3);

    // Do the same for each camera state
    for (auto n = 0; n < N; ++n) {
        dq = errorQuaternion(state_error.segment<3>(12 + 6 * n));
        cameraQuaternion(n) = dq * cameraQuaternion(n);
        cameraPosition(n) += state_error.segment<3>(15 + 6 * n);
    }
}

void EKF::qrDecomposition(VectorXd &r, MatrixXd &H, MatrixXd &R) {
    auto qr = H.householderQr();
    const auto m = H.rows();
    const auto n = H.cols();
    if (m == 0 || n == 0 || n > m) {
        return;
    }

    MatrixXd Q1 = qr.householderQ() * MatrixXd::Identity(m, n);
    H = qr.matrixQR().topLeftCorner(n, n).template triangularView<Eigen::Upper>();

    r = Q1.transpose() * r;
    R = Q1.transpose() * R * Q1;
}

}  // namespace msckf
