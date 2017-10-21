//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#ifndef MSCKF_TEST_HELPERS_HPP
#define MSCKF_TEST_HELPERS_HPP

#include <random>
#include <gtest/gtest.h>

#include "msckf/types.hpp"


namespace msckf {

inline double gaussianNoise(double stddev) {
    static std::default_random_engine generator{std::random_device{}()};
    static std::normal_distribution<> normal{0, 1};

    return stddev * normal(generator);
}

inline void CHECK_VECTOR_NEAR(const Vector3d &expected, const Vector3d &actual, double tol) {
    EXPECT_NEAR(expected(0), actual(0), tol);
    EXPECT_NEAR(expected(1), actual(1), tol);
    EXPECT_NEAR(expected(2), actual(2), tol);
}

inline void CHECK_VECTOR_DOUBLE(const Vector3d &expected, const Vector3d &actual) {
    EXPECT_DOUBLE_EQ(expected(0), actual(0));
    EXPECT_DOUBLE_EQ(expected(1), actual(1));
    EXPECT_DOUBLE_EQ(expected(2), actual(2));
}

inline bool VectorsNear(const VectorXd &v1, const VectorXd &v2) {
    // The absolute comparison of isMuchSmallerThan works better for us than relative comparison of isApprox
    return (v1 - v2).isMuchSmallerThan(1.0, 1e-9);
}
// Need a separate name to take tolerance argument, otherwise gtest compile errors
inline bool VectorsNearTol(const VectorXd &v1, const VectorXd &v2, double tolerance) {
    // The absolute comparison of isMuchSmallerThan works better for us than relative comparison of isApprox
    return (v1 - v2).isMuchSmallerThan(1.0, tolerance);
}

inline bool MatricesNear(const MatrixXd &v1, const MatrixXd &v2) {
    // The absolute comparison of isMuchSmallerThan works better for us than relative comparison of isApprox
    return (v1 - v2).isMuchSmallerThan(1.0, 1e-9);
}

}  // namespace msckf

#endif //MSCKF_TEST_HELPERS_HPP
