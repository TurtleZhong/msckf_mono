//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#ifndef MSCKF_ESTIMATION_HPP
#define MSCKF_ESTIMATION_HPP

#include <unsupported/Eigen/LevenbergMarquardt>
#include "msckf/types.hpp"

namespace msckf {

// Functor representing measurement of a single feature from a single camera pose
struct MeasurementModel : public Eigen::DenseFunctor<double> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MeasurementModel(VectorOfVector2d measurements,
                     const VectorOfMatrix3d &camera_rotations,
                     const VectorOfVector3d &camera_positions);

    // Calculate residuals for each frame given the parameters x. Return 0.
    int operator()(const Vector3d &x, VectorXd &fvec);

    // Calculate Jacobian matrix. Return 0.
    int df(const Vector3d &x, MatrixXd &fjac);

    // Helper function to get h_i(alpha, beta, rho) for given camera frame
    Vector3d hFromParams(const Vector3d &x, int i) const;

    const size_t M;
    VectorOfVector2d measurements_;
    VectorOfMatrix3d local_rotations_;
    VectorOfVector3d local_positions_;
};

// Convert 3d position in camera frame to idealized measurement
Vector2d measurementFromPosition(const Vector3d &position);

// Estimate feature position given M ideal pixel measurements and M camera pose estimates
Vector3d estimateFeaturePosition(const VectorOfVector2d &measurements,
                                 const VectorOfMatrix3d &camera_rotation_estimates,
                                 const VectorOfVector3d &camera_position_estimates,
                                 VectorXd &residuals);

// Estimate 3D feature position in the frame of camera 1 using least squares,
// given two camera measurements and the transform to camera frame 2
Vector3d triangulateFromTwoCameraPoses(const Vector2d &measurement1,
                                       const Vector2d &measurement2,
                                       const Matrix3d &rotation1to2,
                                       const Vector3d &translation1to2);

}  // namespace msckf

#endif //MSCKF_ESTIMATION_HPP
