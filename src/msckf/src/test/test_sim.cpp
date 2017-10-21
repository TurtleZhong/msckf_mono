//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#include <gtest/gtest.h>
#include <cmath>
#include <fstream>

#include "msckf/estimation.hpp"
#include "msckf/util.hpp"
#include "msckf/EKF.hpp"
#include "virtual_drive.hpp"

namespace msckf {

class SimulationTest : public ::testing::Test {
 protected:
    const double camera_stddev = 2;
    const int n_features = 3000;
    const Vector3d feature_box{400, 250, 250};
    VirtualDrive sim = VirtualDrive{n_features, 0, camera_stddev, feature_box};
    EKF ekf = EKF{sim.motion.orientationAtTime(0),
                  sim.motion.positionAtTime(0)};

    // Simulate IMU measurements at 50 Hz and camera measurements at 5 Hz
    const double duration = 15.0;
    const double dt = 0.02;
    const int camera_step = 10;

    // Noise added to inputs
    const double vel_stddev = 1.0;
    const double ang_vel_stddev = 5 * M_PI / 180;

    void SetUp() override {
        sim.motion.path_box_ << 100, 30, 30;
        ekf.p_imu_to_camera = Vector3d{0, 0, 0};
        ekf.q_imu_to_camera = quaternionFromXYZ({M_PI, -M_PI / 2, -M_PI / 2});
        ekf.image_variance = camera_stddev * camera_stddev / 500 / 500; // divide by focal length
        ekf.max_feature_tracks_per_update = 20;

        // Set process noise (todo: improve interface)
        ekf.process_noise.segment(COV_POS, 3) = vel_stddev * Vector3d::Ones();
        ekf.process_noise.segment(COV_BG, 3) = ang_vel_stddev / 10 * Vector3d::Ones();
        ekf.process_noise.segment(COV_BV, 3) = vel_stddev / 5 * Vector3d::Ones();
        ekf.process_noise.segment(COV_ROT, 3) = ang_vel_stddev * Vector3d::Ones();

        // Reset to apply process noise change (todo: improve interface)
        ekf.reset(sim.motion.orientationAtTime(0), sim.motion.positionAtTime(0));
    }

    std::vector<VectorXd> motion_save;
    std::vector<VectorXd> msckf_save;
    std::vector<VectorXd> imu_ekf_save;

    // Save simulation results to csv files in a temporary directory
    void saveToCsv() {
        char name_template[] = "/tmp/msckf_sim_XXXXXX";
        char *dirstr = mkdtemp(name_template);
        auto dir = std::string{dirstr};

        // features.csv: true position of each feature in world frame
        std::ofstream out_file(dir + "/features.csv");
        out_file << std::fixed << std::setprecision(4);

        // Header
        out_file << "id, x, y, z\n";
        auto L = sim.features.size();
        for (auto i = 0 * L; i < L; ++i) {
            const auto &f = sim.features[i];
            out_file << i << ", " << f.position.format(CommaFormat) << "\n";
        }
        out_file.close();

        // motion.csv: true values of motion, and the noisy inputs given
        out_file.open(dir + "/motion.csv");

        // Header
        out_file << "time, x, y, z, rot_x, rot_y, rot_z, "
                 << "vel_x, vel_y, vel_z, rot_vel_x, rot_vel_y, rot_vel_z, "
                 << "vel_input_x, vel_input_y, vel_input_z, "
                 << "rot_input_x, rot_input_y, rot_input_z\n";
        for (auto &m : motion_save) {
            out_file << m.format(CommaFormat) << "\n";
        }
        out_file.close();

        // msckf.csv: estimated values from msckf
        out_file.open(dir + "/msckf.csv");

        // Header
        out_file << "time, x, y, z, rot_x, rot_y, rot_z, "
                 << "cov_x, cov_y, cov_z, cov_rot_x, cov_rot_y, cov_rot_z\n";
        for (auto &m : msckf_save) {
            out_file << m.format(CommaFormat) << "\n";
        }
        out_file.close();

        // imu_only.csv: estimated values from imu-only ekf
        out_file.open(dir + "/imu_only.csv");

        // Header
        out_file << "time, x, y, z, rot_x, rot_y, rot_z, "
                 << "cov_x, cov_y, cov_z, cov_rot_x, cov_rot_y, cov_rot_z\n";
        for (auto &m : imu_ekf_save) {
            out_file << m.format(CommaFormat) << "\n";
        }
        out_file.close();

        // parameters.csv: simulation parameters
        out_file.open(dir + "/parameters.csv");

        // Header
        out_file << "duration, dt, n_features, vel_stddev, rot_vel_stddev, camera_stddev\n";
        out_file << duration << ", " << dt << ", " << n_features << ", "
                 << vel_stddev << ", " << ang_vel_stddev << ", "
                 << camera_stddev << "\n";
        out_file.close();

        std::cerr << "Saved simulation results to " << dir << "/" << std::endl;
    }
};

TEST_F(SimulationTest, SimulateImuOnly) {
    Matrix3d RT = ekf.q_imu_to_camera.matrix().transpose();
    ASSERT_PRED2(VectorsNear, Vector3d::UnitX(), RT * Vector3d::UnitZ());
    ASSERT_PRED2(VectorsNear, -Vector3d::UnitY(), RT * Vector3d::UnitX());
    ASSERT_PRED2(VectorsNear, -Vector3d::UnitZ(), RT * Vector3d::UnitY());

    for (auto k = 1; k < duration / dt; ++k) {
        auto t = k * dt;

        // Give IMU measurement
        auto ang_vel = sim.motion.bodyAngularVelocityAtTime(t);
        auto vel = sim.motion.bodySpeedAtTime(t);

        ekf.estimate(dt, ang_vel, vel);

        auto dq = quaternionFromXYZ(sim.motion.orientationAtTime(t)) * ekf.quaternion().inverse();
        auto aa = Eigen::AngleAxisd{dq};
        auto deg_error = aa.angle() * 180 / M_PI;
        ASSERT_LT(deg_error, 15);
        ASSERT_PRED3(VectorsNearTol, sim.motion.positionAtTime(t), ekf.position(), 3.0);
    }
}

TEST_F(SimulationTest, SimulateMsckf) {

    // Simulate two EKFs, one with only inertial input, and compare
    EKF imu_ekf = EKF{sim.motion.orientationAtTime(0),
                      sim.motion.positionAtTime(0)};
    imu_ekf.p_imu_to_camera = Vector3d{0, 0, 0};
    imu_ekf.q_imu_to_camera = quaternionFromXYZ({M_PI, -M_PI / 2, -M_PI / 2});

    // Set process noise (todo: improve interface)
    imu_ekf.process_noise.segment(COV_POS, 3) = vel_stddev * Vector3d::Ones();
    imu_ekf.process_noise.segment(COV_BG, 3) = ang_vel_stddev /10 * Vector3d::Ones();
    imu_ekf.process_noise.segment(COV_BV, 3) = vel_stddev / 5 * Vector3d::Ones();
    imu_ekf.process_noise.segment(COV_ROT, 3) = ang_vel_stddev * Vector3d::Ones();
    imu_ekf.reset(sim.motion.orientationAtTime(0), sim.motion.positionAtTime(0));

    Vector3d bias_gyro = Vector3d::Zero();
    Vector3d bias_vel = Vector3d::Zero();

    for (auto k = 1; k < duration / dt; ++k) {
        auto t = k * dt;
        auto t_prev = (k - 0.5) * dt;

        // Give IMU measurement with noise
        bias_gyro += Vector3d::Constant(ang_vel_stddev).unaryExpr(&gaussianNoise) / 40;
        bias_vel += Vector3d::Constant(vel_stddev).unaryExpr(&gaussianNoise) / 10;

        Vector3d input_rot_vel = sim.motion.bodyAngularVelocityAtTime(t_prev)
                + Vector3d::Constant(ang_vel_stddev).unaryExpr(&gaussianNoise)
                + bias_gyro;
        Vector3d input_vel = sim.motion.bodySpeedAtTime(t_prev)
                + Vector3d::Constant(vel_stddev).unaryExpr(&gaussianNoise)
                + bias_vel;
        auto pos = sim.motion.positionAtTime(t);
        auto orientation = sim.motion.orientationAtTime(t);

        std::cout << "World ang vel: " << sim.motion.worldAngularVelocityAtTime(t).transpose() * 180 / M_PI
                  << std::endl;
        std::cout << "World vel: " << sim.motion.worldSpeedAtTime(t).transpose() << std::endl;

        std::cout << "Input ang vel deg: " << input_rot_vel.transpose() * 180 / M_PI << std::endl;
        std::cout << "Input vel m/s: " << input_vel.transpose() << std::endl;
        std::cout << "Bias ang vel deg: " << (bias_gyro * 180 / M_PI).format(CommaFormat) << std::endl;
        std::cout << "Bias vel m/s: " << bias_vel.format(CommaFormat) << std::endl;

        ekf.estimate(dt, input_rot_vel, input_vel);
        imu_ekf.estimate(dt, input_rot_vel, input_vel);

        std::cout << "Actual    pos: " << sim.motion.positionAtTime(t).transpose() << std::endl;
        std::cout << "Estimated pos: " << ekf.position().transpose() << std::endl;
        std::cout << "Actual    orien: " << sim.motion.orientationAtTime(t).transpose() * 180 / M_PI << std::endl;

        auto orient = xyzFromQuaternion(ekf.quaternion());
//        orient += Vector3d{M_PI, M_PI, M_PI};

        std::cout << "Estimated orien: " << orient.transpose() * 180 / M_PI << std::endl;

        std::cout << std::endl;

        if (k % camera_step == 0) {
            // time for camera measurement
            auto fmeas = sim.featureMeasurementsAtTime(t);
            ImageSeq seq = static_cast<unsigned int>(k);
            ekf.addCameraFrame(seq);
            ekf.addFeatures(seq, fmeas);
        }

        auto dq = quaternionFromXYZ(sim.motion.orientationAtTime(t)) * ekf.quaternion().inverse();
        auto aa = Eigen::AngleAxisd{dq};
        auto deg_error = aa.angle() * 180 / M_PI;

        dq = quaternionFromXYZ(sim.motion.orientationAtTime(t)) * imu_ekf.quaternion().inverse();
        aa = Eigen::AngleAxisd{dq};
        auto imu_deg_error = aa.angle() * 180 / M_PI;

        const auto &pos_err = ekf.position() - sim.motion.positionAtTime(t);
        const auto &imu_pos_err = imu_ekf.position() - sim.motion.positionAtTime(t);

        std::cout << "Position error full EKF: " << pos_err.norm() << std::endl;
        std::cout << "Position error IMU only: " << imu_pos_err.norm() << std::endl;
        std::cout << "Orientation error full EKF: " << deg_error << std::endl;
        std::cout << "Orientation error IMU only: " << imu_deg_error << std::endl;
        std::cout << std::endl;

//        ASSERT_LT(deg_error, 45);
//        ASSERT_PRED3(VectorsNearTol, sim.motion.positionAtTime(t), ekf.position(), 10.0);

        // Save for later export
        auto motion_to_save = VectorXd{19};
        motion_to_save << t, pos, orientation, sim.motion.bodySpeedAtTime(t), sim.motion.bodyAngularVelocityAtTime(t),
                input_vel, input_rot_vel;
        motion_save.push_back(motion_to_save);

        auto msckf_estimate_to_save = VectorXd{13};
        msckf_estimate_to_save << t, ekf.position(), xyzFromQuaternion(ekf.quaternion()),
                ekf.imuCovariance().diagonal().segment<3>(COV_POS),
                ekf.imuCovariance().diagonal().segment<3>(COV_ROT);
        msckf_save.push_back(msckf_estimate_to_save);

        auto imu_estimate_to_save = VectorXd{13};
        imu_estimate_to_save << t, imu_ekf.position(), xyzFromQuaternion(imu_ekf.quaternion()),
                imu_ekf.imuCovariance().diagonal().segment<3>(COV_POS),
                imu_ekf.imuCovariance().diagonal().segment<3>(COV_ROT);
        imu_ekf_save.push_back(imu_estimate_to_save);
    }


    // Save the simulation results
    saveToCsv();
}

}  // namespace msckf
