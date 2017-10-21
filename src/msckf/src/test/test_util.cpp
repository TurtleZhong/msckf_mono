//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//

#include <gtest/gtest.h>

#include "msckf/util.hpp"
#include "test_helpers.hpp"

namespace msckf {

TEST(UtilTest, QuaternionFromVector) {
    // Give a positive vector from [0, pi/2]
    // Should work for any vector, but Euler angles aren't unique
    // Thus a == xyzFromQuaternion(quaternionFromXYZ(a)) not true for all
    Vector3d a = (Vector3d::Random() + Vector3d::Ones()) * M_PI / 4;

    auto sx = sin(a.x());
    auto cx = cos(a.x());
    auto sy = sin(a.y());
    auto cy = cos(a.y());
    auto sz = sin(a.z());
    auto cz = cos(a.z());

    Matrix3d expected;
    expected << cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx,
            sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx,
            -sy, cy * sx, cy * cx;

    auto q = quaternionFromXYZ(a);
    EXPECT_PRED2(MatricesNear, expected, q.matrix());

    auto a2 = xyzFromQuaternion(q);

    EXPECT_PRED2(VectorsNear, a, a2);
}

TEST(UtilTest, CrossMatrix) {
    auto a = Vector3d::Random().eval();
    auto b = Vector3d::Random().eval();

    EXPECT_EQ(a.cross(b), crossMatrix(a) * b);
    EXPECT_EQ(b.cross(a), crossMatrix(b) * a);
}

TEST(UtilTest, ErrorQuaternion) {
    const auto TOLERANCE = 1e-3;
    // todo: more reliable check
    auto t1 = Vector3d{0, 0, 1.0} * M_PI / 180;
    auto t2 = Vector3d{0.5, 1.5, 0.8} * M_PI / 180;
    auto angle_error = t2 - t1;
    auto q1 = quaternionFromXYZ(t1);
    auto q2 = quaternionFromXYZ(t2);

    auto dq = errorQuaternion(angle_error);

    EXPECT_DOUBLE_EQ(1.0, dq.norm());

    auto q3 = Quaterniond{dq * q1};
    auto t3 = xyzFromQuaternion(q3);

    EXPECT_NEAR(t2(0), t3(0), TOLERANCE);
    EXPECT_NEAR(t2(1), t3(1), TOLERANCE);
    EXPECT_NEAR(t2(2), t3(2), TOLERANCE);
    EXPECT_DOUBLE_EQ(1.0, q3.norm());
}

TEST(UtilTest, InverseDepthParams) {
    auto a = Vector3d{0.5, -6.0, -3.5};

    // inverseDepthParams() divides by the *absolute value* of the third component
    // this is useful because optimizer can converge to negative depth
    auto expected = Vector3d{0.5/3.5, -6.0/3.5, 1.0/3.5};

    auto result = inverseDepthParams(a);

    EXPECT_PRED2(VectorsNear, expected, result);
}

TEST(UtilTest, SizeOfNestedContainers) {
    auto v = std::vector<std::vector<double>>{};
    v.push_back({1.0, 1.1, 1.2});
    v.push_back({2.0});
    v.push_back({});

    EXPECT_EQ(4, sizeOfNestedContainers(v));
}

TEST(UtilTest, IdealizeMeasurement) {
    auto p = Vector2d{230, 100};
    auto camera = CameraParameters{700.1, 750.2, 610.1, 120.5};

    auto r = idealizeMeasurement(p, camera);

    EXPECT_DOUBLE_EQ((p(0) - 610.1) / 700.1, r(0));
    EXPECT_DOUBLE_EQ((p(1) - 120.5) / 750.2, r(1));
}

}  // namespace msckf
