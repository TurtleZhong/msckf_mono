//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#include <gtest/gtest.h>
#include <cmath>

#include "msckf/EKF.hpp"
#include "test_helpers.hpp"

namespace msckf {

class EkfTest : public ::testing::Test {
 protected:
    const Vector3d INITIAL_ROTATION = Vector3d{10, 20, 30} * M_PI / 180;
    const Quaterniond INITIAL_QUATERNION = quaternionFromXYZ(INITIAL_ROTATION);
    const Vector3d INITIAL_POSITION{21, 22, 23};
    EKF ekf{INITIAL_ROTATION, INITIAL_POSITION};

    const Vector3d P_IMU_TO_CAMERA{1.1, 2.2, 3.3};
    const Quaterniond Q_IMU_TO_CAMERA{Quaterniond::UnitRandom()};

    void SetUp() override {
        ekf.p_imu_to_camera = P_IMU_TO_CAMERA;
        ekf.q_imu_to_camera = Q_IMU_TO_CAMERA;
        //ekf.process_covariance = ImuStateVector{};
    }
};

TEST_F(EkfTest, ReadWriteState) {
    EXPECT_DOUBLE_EQ(21, ekf.position()(0));

    ekf.position()(0) = 99;
    EXPECT_DOUBLE_EQ(99, ekf.position()(0));
    EXPECT_DOUBLE_EQ(99, ekf.imuState()(10));

    ekf.position() = Vector3d{1, 2, 3};
    EXPECT_DOUBLE_EQ(1, ekf.position()(0));
    EXPECT_DOUBLE_EQ(1, ekf.imuState()(10));
}

TEST_F(EkfTest, ReadWriteRotationState) {
    EXPECT_DOUBLE_EQ(INITIAL_QUATERNION.x(), ekf.quaternion_vector()(0));

    ekf.quaternion_vector()(0) = 0.2;
    EXPECT_DOUBLE_EQ(0.2, ekf.quaternion_vector()(0));
// note: will not be normalized with direct assignment

    ekf.addCameraFrame(12);

    auto q = quaternionFromXYZ({1.1, 2.2, 3.3});
    ekf.cameraQuaternion(0) = q;
    EXPECT_PRED2(MatricesNear, q.matrix(), ekf.cameraRotation(0));
}

TEST_F(EkfTest, ReadWriteCovarianceBlocks) {
    ekf.imuCovariance()(4, 6) = 123.123;
    EXPECT_PRED2(MatricesNear, ekf.covariance(), ekf.imuCovariance());

    ekf.addCameraFrame(12);
    EXPECT_DOUBLE_EQ(123.123, ekf.imuCovariance()(4, 6));
    EXPECT_DOUBLE_EQ(123.123, ekf.covariance()(4, 6));
}

TEST_F(EkfTest, EstimateState) {
    // Propagate state with no input; expect no change
    ekf.estimate(0.1, {0, 0, 0}, {0, 0, 0});
    EXPECT_DOUBLE_EQ(INITIAL_POSITION(0), ekf.position()(0));
    EXPECT_DOUBLE_EQ(INITIAL_QUATERNION.x(), ekf.quaternion_vector()(0));

    auto res = xyzFromQuaternion(ekf.quaternion());
    EXPECT_PRED2(VectorsNear, INITIAL_ROTATION, res);

    // Propagate state with velocity input
    ekf.estimate(0.1, {0, 0, 0}, {-100, 0, 0});

    auto local_x = 0.1 * -100;
    auto global_x = local_x * cos(INITIAL_ROTATION(2));
    auto global_y = local_x * sin(INITIAL_ROTATION(2));

    Vector3d expected = INITIAL_POSITION + INITIAL_QUATERNION.matrix().transpose() * Vector3d{local_x, 0, 0};
    EXPECT_PRED2(VectorsNear, expected, ekf.position());
    EXPECT_PRED2(VectorsNear, INITIAL_QUATERNION.coeffs(), ekf.quaternion_vector());
}

TEST_F(EkfTest, EstimateStateRotationTrivial) {
    // Propagate state with rotation input about body z axis
    const Vector3d input_rot_vel = Vector3d::Random() * 0.1 * M_PI;
    const auto dt = 0.05;
    ekf.estimate(dt, input_rot_vel, {0, 0, 0});

    // Calculate expected rotation (should work for small time step)
    Quaterniond dq{Eigen::AngleAxisd{input_rot_vel.norm() * dt,
                                     input_rot_vel / input_rot_vel.norm()}};
    Quaterniond expected_q = INITIAL_QUATERNION * dq;
    auto expected = xyzFromQuaternion(expected_q);

    auto res = xyzFromQuaternion(ekf.quaternion());
    auto TOLERANCE = 1e-5;

    EXPECT_PRED3(VectorsNearTol, expected, res, TOLERANCE);
    EXPECT_DOUBLE_EQ(1, ekf.quaternion().norm());
    EXPECT_PRED2(VectorsNear, INITIAL_POSITION, ekf.position());

}

//todo: nontrivial rotation test

TEST_F(EkfTest, InitialCameraState) {
    EXPECT_EQ(0, ekf.nCameraPoses());
    EXPECT_EQ(0, ekf.cameraState().size());
    EXPECT_EQ(0, ekf.cameraCameraCovariance().size());
}

TEST_F(EkfTest, AddCameraFrame) {
    ekf.addCameraFrame(0);

    const auto p_expected = Vector3d{
            INITIAL_POSITION + INITIAL_QUATERNION.matrix().transpose() * P_IMU_TO_CAMERA};
    const auto q_expected = Quaterniond{Q_IMU_TO_CAMERA * INITIAL_QUATERNION};

    // Check new pose
    EXPECT_EQ(1, ekf.nCameraPoses());
    EXPECT_PRED2(VectorsNear, p_expected, ekf.cameraPosition(0));
    EXPECT_PRED2(VectorsNear, q_expected.coeffs(), ekf.cameraQuaternion(0).coeffs());

    // Check imu state is unaffected
    SCOPED_TRACE("IMU state corrupted by adding camera frame");
    EXPECT_PRED2(VectorsNear, INITIAL_POSITION, ekf.position());

    // Check covariance
    EXPECT_EQ(6, ekf.cameraCameraCovariance().rows());
    EXPECT_EQ(6, ekf.cameraCameraCovariance().cols());
    EXPECT_EQ(12, ekf.imuCameraCovariance().rows());
    EXPECT_EQ(6, ekf.imuCameraCovariance().cols());
}

TEST_F(EkfTest, Jacobian) {
    auto J = ekf.jacobian();

    EXPECT_EQ(6, J.rows());
    EXPECT_EQ(12, J.cols());

    ekf.addCameraFrame(0);

    J = ekf.jacobian();
    EXPECT_EQ(6, J.rows());
    EXPECT_EQ(12 + 6, J.cols());

    // check that covariance expanded
    EXPECT_EQ(6, ekf.cameraCameraCovariance().rows());
    EXPECT_EQ(6, ekf.cameraCameraCovariance().cols());

    ekf.addCameraFrame(1);

    EXPECT_EQ(12, ekf.cameraCameraCovariance().rows());
    EXPECT_EQ(12, ekf.cameraCameraCovariance().cols());

    // todo: more tests
}

TEST_F(EkfTest, CameraIndexing) {
    EXPECT_EQ(0, ekf.nCameraPoses());
    EXPECT_THROW(ekf.nFromImageSeq(0), std::out_of_range);

    ekf.addCameraFrame(52);
    ekf.addCameraFrame(99);

    EXPECT_EQ(2, ekf.nCameraPoses());
    EXPECT_EQ(0, ekf.nFromImageSeq(52));
    EXPECT_EQ(1, ekf.nFromImageSeq(99));
    EXPECT_THROW(ekf.nFromImageSeq(53), std::out_of_range);
    // todo: extend when N_max implemented
}

TEST_F(EkfTest, UpdateEkfEmpty) {
    // Test calling updateEkf with empty matrices
    VectorXd r{0};
    MatrixXd H{0, 12};  // still must have 12 + 6N columns...
    MatrixXd R{0, 0};
    EXPECT_NO_THROW(ekf.updateEkf(r, H, R));
}

TEST_F(EkfTest, UpdateEkf) {
    // todo: For now just check there is no crash
    // and no unexpected effect on state size
    ekf.addCameraFrame(0);

    EXPECT_EQ(7, ekf.cameraState().size());

    VectorXd r{0};
    MatrixXd H{0, 18};  // must have 12 + 6N columns...
    MatrixXd R{0, 0};
    EXPECT_NO_THROW(ekf.updateEkf(r, H, R));

    EXPECT_EQ(7, ekf.cameraState().size());
}

}  // namespace msckf
