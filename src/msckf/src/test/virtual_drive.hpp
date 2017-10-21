//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#ifndef MSCKF_VIRTUAL_DRIVE_HPP
#define MSCKF_VIRTUAL_DRIVE_HPP

#include "virtual_camera.hpp"

namespace msckf {

// The lists of measurements and camera poses for each feature detected
struct FeatureData {
    Vector3d position;  // actual position in global frame
    VectorOfVector2d measurements;
    VectorOfMatrix3d camera_rotations;
    VectorOfVector3d camera_positions;
};

// A curvy path
class VirtualMotion {
    const int num_periods = 2;  // number of cycles of the curve to fit inside the box
    const double duration_;  // time to reach far x wall
    const double f_;  // frequency of the sine curve

 public:
    Vector3d path_box_{100, 2, 0.5};  // 1, left-right, and up-down limits of the path

    explicit VirtualMotion(double duration) :
            duration_{duration},
            f_{2 * M_PI * num_periods / duration} {}

    Vector3d positionAtTime(double t) {
        return path_box_.asDiagonal() * Vector3d{t / duration_, cos(f_ * t), sin(f_ * t)};
    }

    Vector3d worldSpeedAtTime(double t) {
        return path_box_.asDiagonal() * Vector3d{1 / duration_, -f_ * sin(f_ * t), f_ * cos(f_ * t)};
    }

    Vector3d orientationAtTime(double t) {
        // Define an orientation independent from motion, for now
        return {sin(f_ * t) / 10, sin(f_ * t) / 5, sin(f_ * t) / 3};
    }

    Matrix3d cameraRotationAtTime(double t) {
        // Rotate so camera Z points in body X
        auto q = quaternionFromXYZ(orientationAtTime(t));
        auto qcam = quaternionFromXYZ({M_PI, -M_PI / 2, -M_PI / 2}).matrix();
        return qcam * q;
    }

    Vector3d worldAccelerationAtTime(double t) {
        return path_box_.asDiagonal() * Vector3d{0, -f_ * f_ * cos(f_ * t), -f_ * f_ * sin(f_ * t)};
    }

    Vector3d worldAngularVelocityAtTime(double t) {
        // analytical derivative of orientationAtTime
        return Vector3d{cos(f_ * t) / 10, cos(f_ * t) / 5, cos(f_ * t) / 3} * f_;
    }

    Vector3d bodyAngularVelocityAtTime(double t) {
        Matrix3d R = quaternionFromXYZ(orientationAtTime(t)).matrix();
        return R * worldAngularVelocityAtTime(t);
    }

    Vector3d bodySpeedAtTime(double t) {
        Matrix3d R = quaternionFromXYZ(orientationAtTime(t)).matrix();
        return R * worldSpeedAtTime(t);
    }

};

// A class for collecting a virtual set of measurements for testing
class VirtualDrive {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    const double duration = 10;
    const double camera_noise_;

 public:
    VirtualMotion motion{duration};
    std::vector<FeatureData> features;  // positions in global frame

    FeatureList featureMeasurementsAtTime(double t) {
        auto position = motion.positionAtTime(t);
        auto worldSpeed = motion.worldSpeedAtTime(t);
        auto rotation = motion.cameraRotationAtTime(t);
        const auto camera = VirtualCamera{rotation, position, camera_noise_};

        std::vector<ImageFeature> measurements;

        auto L = features.size();
        for (auto i = 0 * L; i < L; ++i) {
            ImageFeature m;
            if (camera.measureFeature(features[i].position, m.point)) {
                m.id = i;
                measurements.push_back(m);
            }
        }

        return measurements;
    }

    VirtualDrive(int n_features,
                 int n_cameras,
                 double camera_stddev,
                 const Vector3d &feature_box = {200, 5, 5}) :
            features(n_features),
            camera_noise_{camera_stddev} {
        // Generate uniformly distributed features
        for (auto &f : features) {
            f.position = 0.5 * feature_box.asDiagonal() * Vector3d::Random();
            f.position(0) += 0.5 * feature_box(0);
        }

        // For convenience, generate list of feature measurements
        for (auto i = 0; i < n_cameras; ++i) {
            // Get camera pose along the path
            const auto t = duration * i / n_cameras;

            auto measurements = featureMeasurementsAtTime(t);

            for (auto &m : measurements) {
                auto &f = features[m.id];
                f.measurements.push_back(m.point);
                f.camera_rotations.push_back(motion.cameraRotationAtTime(t));
                f.camera_positions.push_back(motion.positionAtTime(t));
            }
        }
    }
};

}  // namespace msckf

#endif  // MSCKF_VIRTUAL_DRIVE_HPP
