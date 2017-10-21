//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#include <gtest/gtest.h>
#include <cmath>

#include "msckf/estimation.hpp"
#include "msckf/util.hpp"
#include "virtual_drive.hpp"


namespace msckf {

// This test fixture has two virtual cameras and one feature
class TriangulationTest : public ::testing::Test {
    const double CAMERA_STDDEV = 0.5;
 protected:
    // Define an arbitrary feature position in the global frame
    Vector3d X{50.0, 3.5, 1.1};

    // Define two arbitrary cameras roughly aimed at the feature
    // Note: if the camera moves directly at the feature, the algorithm cannot estimate depth.
    // The greater the baseline, the more noise it can tolerate.
    Vector3d p1{3, 2.1, 1.5};  // camera 1 position
    Vector3d p2{10, 4.3, 1.4};  // camera 2 position
    Matrix3d R1{quaternionFromXYZ({-M_PI / 2 - 0.1, 0.03, -M_PI / 2 + 0}).inverse()};  // camera 1 rotation
    Matrix3d R2{quaternionFromXYZ({-M_PI / 2 + 0.1, -0.02, -M_PI / 2 + 0}).inverse()};  // camera 2 rotation

    // Create virtual cameras
    VirtualCamera C1{R1, p1, CAMERA_STDDEV};
    VirtualCamera C2{R2, p2, CAMERA_STDDEV};

    // Compute transform between the two camera poses
    Matrix3d C1to2 = R2 * R1.inverse();
    Vector3d p1to2 = R1 * (p2 - p1);

    Vector2d z1, z2;

    TriangulationTest() {
        // Take idealized measurement with some noise
        C1.measureFeature(X, z1);
        C2.measureFeature(X, z2);
    }
};

TEST_F(TriangulationTest, Triangulate) {
    // The correct result is the feature position in frame of camera 1
    auto expected = R1 * (X - p1);

    // Estimate the position from noisy measurements
    auto res = triangulateFromTwoCameraPoses(z1, z2, C1to2, p1to2);

    const auto TOLERANCE = 1.0;  // We just need an estimate and expect some error
    EXPECT_NEAR(expected(0), res(0), TOLERANCE);
    EXPECT_NEAR(expected(1), res(1), TOLERANCE);
    EXPECT_NEAR(expected(2), res(2), TOLERANCE);
}

TEST_F(TriangulationTest, OptimizeSimple) {
    // Here we test the MeasurementModel optimization directly.
    // The correct result is the feature position in frame of camera 1
    auto expected = R1 * (X - p1);

    VectorOfVector2d measurements{z1, z2};
    VectorOfMatrix3d camera_rotations{R1, R2};
    VectorOfVector3d camera_positions{p1, p2};

    auto model = MeasurementModel{measurements, camera_rotations, camera_positions};
    Eigen::LevenbergMarquardt<MeasurementModel> lm{model};

    // The parameters work in local coordinates of the first camera frame
    auto params = VectorXd{3};  // Note params must be VectorXd, not fixed size
    params << inverseDepthParams({1, 1, 20}); // rough guess
    lm.minimize(params);
    auto info = lm.info();
    auto estimated_pos = inverseDepthParams(params);

    EXPECT_EQ(Eigen::Success, info);
    // Expect some error as there is noise
    EXPECT_PRED3(VectorsNearTol, expected, estimated_pos, 0.2);
}

// This test feature has N features and M cameras
// For now the camera interpolates a sinusoidal path through a uniform cloud of features
class EstimationTest : public ::testing::Test {

};


TEST_F(EstimationTest, OptimizeFeaturePositions) {
    /* Here we test the MeasurementModel optimization directly.
     * Don't use estimateFeaturePosition() as it would start with an initial guess,
     * and the initial guess would be perfect because there is no noise. */
    const auto &features = VirtualDrive{100, 100, 0.0}.features;

    for (auto &f : features) {
        if (f.measurements.size() > 2) {

            auto model = MeasurementModel{f.measurements, f.camera_rotations, f.camera_positions};
            Eigen::LevenbergMarquardt<MeasurementModel> lm{model};

            // The parameters work in local coordinates of the first camera frame
            auto local_pos = f.camera_rotations.front() * (f.position - f.camera_positions.front());
            auto params = VectorXd{3};  // Note params must be VectorXd, not fixed size
            params << 1, 1, 1;  // Even a terrible guess should converge
            lm.minimize(params);
            auto info = lm.info();
            auto estimated_pos = inverseDepthParams(params);

            EXPECT_EQ(Eigen::Success, info);
            EXPECT_PRED2(VectorsNear, local_pos, estimated_pos);
        }
    }
}

TEST_F(EstimationTest, EstimateFeaturePositions) {
    const auto MEASUREMENT_NOISE = 0.5;
    const auto &features = VirtualDrive{200, 100, MEASUREMENT_NOISE}.features;
    const auto OUTLIER_TOLERANCE = 10.0;
    auto total_se = 0.0;
    auto n_outliers = 0, n_estimated = 0;
    Eigen::IOFormat fmt{Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ";\n", "", "", "[", "]"};

    for (auto &f : features) {
        // The correct result is the feature position in frame of camera 1

        if (f.measurements.size() > 5) {
            VectorXd residuals;
            auto estimated_pos = estimateFeaturePosition(f.measurements, f.camera_rotations, f.camera_positions,
                                                         residuals);
            auto estimated_local = f.camera_rotations[0] * (estimated_pos - f.camera_positions[0]);

            auto distance = (estimated_pos - f.position).norm();
            if (distance > OUTLIER_TOLERANCE) {
                auto expected_local = f.camera_rotations[0] * (f.position - f.camera_positions[0]);

                std::cerr << "Feature " << f.position.transpose().format(fmt)
                          << " estimated at " << estimated_pos.transpose().format(fmt) << std::endl;
                std::cerr << "Local " << expected_local.transpose().format(fmt) <<
                          estimated_local.transpose().format(fmt) << std::endl;
                ++n_outliers;
            }
            total_se += distance * distance;
            ++n_estimated;

            // Make sure we got the right shape of residuals, and check the first one
            EXPECT_EQ(2 * f.measurements.size(), residuals.size());
            auto r1 = f.measurements[0] - measurementFromPosition(estimated_local);
            EXPECT_PRED2(VectorsNear, r1, residuals.head<2>());
        }
    }

    // With noise we can expect some outliers.
    auto outlier_fraction = 1.0 * n_outliers / n_estimated;
    EXPECT_LT(outlier_fraction, 0.10);

    auto rms = std::sqrt(total_se / n_estimated);
    std::cerr << n_outliers << " big outliers out of " << n_estimated
              << " with noise " << MEASUREMENT_NOISE << ". RMS error " << rms << std::endl;
}

}  // namespace msckf
