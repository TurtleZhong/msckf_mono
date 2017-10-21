//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#ifndef MSCKF_UTIL_HPP
#define MSCKF_UTIL_HPP

#include "msckf/types.hpp"

namespace msckf {

inline Vector3d xyzFromQuaternion(const Quaterniond &quaternion) {
    return quaternion.matrix().eulerAngles(2, 1, 0).reverse();
}

inline Quaterniond quaternionFromXYZ(const Vector3d &vector) {
    return AngleAxisd(vector[2], Vector3d::UnitZ()) *
            AngleAxisd(vector[1], Vector3d::UnitY()) *
            AngleAxisd(vector[0], Vector3d::UnitX());
}

// Cross product matrix, a.k.a skew-symmetric representation of rotation input
// vector --> Matrix
inline Eigen::Matrix3d crossMatrix(const Vector3d &v) {
    Eigen::Matrix3d cross;
    cross << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return cross;
}

// Return error quaternion from small angle error vector
inline Quaterniond errorQuaternion(const Vector3d &v) {
    return Quaterniond{1, 0.5 * v(0), 0.5 * v(1), 0.5 * v(2)}.inverse().normalized();
}

// Turn position into inverse-depth parameters or vice-versa
inline Vector3d inverseDepthParams(const Vector3d &v) {
    const auto& p = Vector3d{v(0), v(1), 1.0} / v(2);
    if (p(2) < 0) {
        return -p;
    }
    return p;
}

// Count total size of vector of containers
template<typename T>
typename T::size_type sizeOfNestedContainers(const std::vector<T> &v) {
    auto lambda = [](typename T::size_type a, T b) { return a + b.size(); };
    return std::accumulate(v.begin(), v.end(), 0, lambda);
}

// Idealize a pixel measurement
inline Vector2d idealizeMeasurement(const Vector2d &p, const CameraParameters &c) {
    return Vector2d{(p(0) - c.cx) / c.fx, (p(1) - c.cy) / c.fy};
}

// Flat comma-separated format for Eigen matrices
const static Eigen::IOFormat CommaFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ");

}  // namespace msckf

#endif  // MSCKF_UTIL_HPP
