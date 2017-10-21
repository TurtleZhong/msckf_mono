//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#ifndef MSCKF_VIRTUAL_CAMERA_HPP
#define MSCKF_VIRTUAL_CAMERA_HPP

#include "test_helpers.hpp"

namespace msckf {

using ProjectionMatrix = Eigen::Matrix<double, 3, 4>;

// Virtual camera used to take measurements
class VirtualCamera {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
    // Construct camera given its pose in the world frame
    VirtualCamera(Matrix3d rotation, Vector3d position, double noise_stddev,
                  double fx = 500, double fy = 500, double cx = 640 / 2, double cy = 480 / 2) :
            rotation{rotation},
            position{position},
            noise_stddev{noise_stddev},
            fx{fx}, fy{fy}, cx{cx}, cy{cy} {
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        ProjectionMatrix Rt;  // extrinsic matrix
        Rt << rotation, -rotation * position;
        P = K * Rt;
    }

    // Output *idealized* feature measurement given feature in the world frame
    // Return true if feature is visible (otherwise output measurement is impossible)
    // Gaussian noise is added to the measurement before it is idealized
    bool measureFeature(const Vector3d &feature_position,
                        Vector2d &ideal_measurement) const {
        auto noise = Vector2d{Vector2d::Constant(noise_stddev).unaryExpr(&gaussianNoise)};
        auto X = Eigen::Vector4d{};
        X << feature_position, 1;

        // Take measurement and add noise
        auto scaled_meas = Vector3d{P * X};
        auto z = scaled_meas(2);

        auto meas = Vector2d{scaled_meas(0) / z, scaled_meas(1) / z};
        meas += noise;

        // Idealize and set output
        ideal_measurement = Vector2d{(meas(0) - cx) / fx, (meas(1) - cy) / fy};

        return isInView(meas, z);
    }

    const double fx, fy, cx, cy;
    const Matrix3d rotation;  // world to camera rotation
    const Vector3d position;  // world to camera translation
 private:
    Matrix3d K;  // intrinsic calibration matrix
    ProjectionMatrix P;
    double noise_stddev;

    // Check if (non-idealized) pixel measurement is actually visible
    bool isInView(const Vector2d &m, double z) const {
        const auto x = m(0), y = m(1);
        return (x > 0 && x < cx * 2 && y > 0 && y < cy * 2 && z > 0);
    }
};

}  // namespace msckf

#endif  // MSCKF_VIRTUAL_CAMERA_HPP
