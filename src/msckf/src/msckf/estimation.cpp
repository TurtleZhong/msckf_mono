//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#include "msckf/estimation.hpp"

#include <unsupported/Eigen/LevenbergMarquardt>
#include "msckf/util.hpp"
namespace msckf {

Vector3d estimateFeaturePosition(const VectorOfVector2d &measurements,
                                 const VectorOfMatrix3d &camera_rotation_estimates,
                                 const VectorOfVector3d &camera_position_estimates,
                                 VectorXd &residuals) {
    // We have one feature's measurements in M images and M camera pose estimates in the global frame
    const auto M = measurements.size();
    assert(M == camera_rotation_estimates.size());
    assert(M == camera_position_estimates.size());
    assert(M > 1);

    /* Follow the approach outlined in the Appendix of Mourikis and Roumeliotis
     * Put everything in terms of three parameters representing the camera-frame feature position
     * in just the first camera frame. We estimate these parameters using Levenberg-Marquardt.
     */
    auto model = MeasurementModel{measurements, camera_rotation_estimates, camera_position_estimates};

    Eigen::LevenbergMarquardt<MeasurementModel> lm{model};

    // Generate initial guess from first and last feature instance
    const auto &m1 = measurements.front();
    const auto &m2 = measurements.back();
    const auto &R1 = camera_rotation_estimates.front();
    const auto &R2 = camera_rotation_estimates.back();
    const auto &p1 = camera_position_estimates.front();
    const auto &p2 = camera_position_estimates.back();
    const auto pos_guess = triangulateFromTwoCameraPoses(m1, m2, R2 * R1.inverse(), R1 * (p2 - p1));

    // Convert this estimated position to the parameters alpha, beta, rho
    // Note: a dymamic vector seems to make the LevenbergMarquardt compile more happily
    auto params = VectorXd{inverseDepthParams(pos_guess)};

    auto status = lm.minimize(params);

    // Convert back to position in global frame
    auto pos = inverseDepthParams(params);
    auto global_pos = Vector3d{R1.transpose() * pos + p1};

    // Output the residuals by calling the function used in the optimization
    residuals.resize(2 * M);
    model(params, residuals);

    return global_pos;
}

Vector3d triangulateFromTwoCameraPoses(const Vector2d &measurement1,
                                       const Vector2d &measurement2,
                                       const Matrix3d &rotation1to2,
                                       const Vector3d &translation1to2) {
    // Use the method in Clement's paper
    // Unit direction vectors from each camera centre to the feature
    Vector3d dir1, dir2;
    dir1 << measurement1, 1;
    dir2 << measurement2, 1;
    dir1.normalize();
    dir2.normalize();

    // Put in the form Ax = b
    MatrixXd A{3, 2};
    A << dir1, -rotation1to2.transpose() * dir2;
    const auto &b = translation1to2;

    // Calculate A\b
    Vector2d x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // Return position in first frame
    return x(0) * dir1;
}

Vector2d measurementFromPosition(const Vector3d &position) {
    return position.head<2>() / position(2);
}

Vector3d MeasurementModel::hFromParams(const Vector3d &x, int i) const {
    const auto &C = local_rotations_[i];
    const auto &p = local_positions_[i];
    return C * Vector3d{x(0), x(1), 1} + x(2) * p;
}

int MeasurementModel::operator()(const Vector3d &x, VectorXd &fvec) {
    // Calculate residuals for each feature measurement i = 1..M
    // Note the residuals are 2D, so length of fvec is 2M.

    for (auto i = 0; i < M; ++i) {
        const auto &z = measurements_[i];

        // h is the vector of the functions h_i1, h_i2, h_i3 in Mourikis and Roumeliotis
        auto h = hFromParams(x, i);

        auto meas = measurementFromPosition(h);

        auto error = Vector2d{z - meas};
        fvec(2 * i) = error(0);
        fvec(2 * i + 1) = error(1);
    }
    return 0;
}

int MeasurementModel::df(const Vector3d &x, MatrixXd &fjac) {
    // Calculate the Jacobian in 2*3 blocks for each measurement
    for (auto i = 0; i < M; ++i) {
        const auto &C = local_rotations_[i];
        const auto &p = local_positions_[i];
        auto h = hFromParams(x, i);

        // Write out the analytical Jacobian
        // These temporary matrices are just for convenience writing it out
        Eigen::Matrix<double, 2, 3> J1, J2;
        J1 << C.topLeftCorner<2, 2>(), p.head<2>();
        J2 << h(0) * C(2, 0), h(0) * C(2, 1), h(0) * p(2),
                h(1) * C(2, 0), h(1) * C(2, 1), h(1) * p(2);
        fjac.block<2, 3>(2 * i, 0) = (-J1 + J2 / h(2)) / h(2);
    }
    return 0;
}

MeasurementModel::MeasurementModel(VectorOfVector2d measurements,
                                   const VectorOfMatrix3d &camera_rotations,
                                   const VectorOfVector3d &camera_positions) :
        Eigen::DenseFunctor<double>{3, static_cast<int>(2 * measurements.size())},
        M{measurements.size()},
        measurements_{measurements} {
    // We have M camera pose estimates in the global frame.
    assert(M == camera_rotations.size());
    assert(M == camera_positions.size());
    local_rotations_.reserve(M);
    local_positions_.reserve(M);

    // Pre-compute the poses in the frame of the first camera pose
    for (auto i = 0; i < M; ++i) {
        // Get rotation from 1st frame to ith frame
        local_rotations_.emplace_back(camera_rotations[i] * camera_rotations[0].transpose());
        // Get translation from 1st frame to ith frame
        local_positions_.emplace_back(camera_rotations[i] * (camera_positions[i] - camera_positions[0]));
    }
}

}  // namespace msckf
