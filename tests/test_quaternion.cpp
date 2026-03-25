#include <gtest/gtest.h>
#include "utils/quaternion.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

TEST(QuaternionTest, DefaultConstructor) {
    Quaternion q;
    EXPECT_DOUBLE_EQ(q.w, 1.0);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_DOUBLE_EQ(q.z, 0.0);
}

TEST(QuaternionTest, ParameterizedConstructor) {
    Quaternion q(1.0, 2.0, 3.0, 4.0);
    EXPECT_DOUBLE_EQ(q.w, 1.0);
    EXPECT_DOUBLE_EQ(q.x, 2.0);
    EXPECT_DOUBLE_EQ(q.y, 3.0);
    EXPECT_DOUBLE_EQ(q.z, 4.0);
}

TEST(QuaternionTest, Magnitude) {
    Quaternion q(1.0, 0.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(q.magnitude(), 1.0);
}

TEST(QuaternionTest, Identity) {
    Quaternion q = Quaternion::identity();
    EXPECT_DOUBLE_EQ(q.w, 1.0);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_DOUBLE_EQ(q.z, 0.0);
}

TEST(QuaternionTest, FromEuler) {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = M_PI / 2;  // 90 degrees

    Quaternion q = Quaternion::from_euler(roll, pitch, yaw);

    // 90° yaw rotation around Y-axis
    EXPECT_NEAR(q.w, std::cos(M_PI / 4), 1e-10);
    EXPECT_NEAR(q.magnitude(), 1.0, 1e-10);
}

TEST(QuaternionTest, FromAxisAngle) {
    Vec3 axis(0.0, 1.0, 0.0);  // Y-axis
    double angle = M_PI / 2;     // 90 degrees

    Quaternion q = Quaternion::from_axis_angle(axis, angle);

    EXPECT_NEAR(q.magnitude(), 1.0, 1e-10);
}

TEST(QuaternionTest, Conjugate) {
    Quaternion q(1.0, 2.0, 3.0, 4.0);
    Quaternion conj = q.conjugate();

    EXPECT_DOUBLE_EQ(conj.w, 1.0);
    EXPECT_DOUBLE_EQ(conj.x, -2.0);
    EXPECT_DOUBLE_EQ(conj.y, -3.0);
    EXPECT_DOUBLE_EQ(conj.z, -4.0);
}

TEST(QuaternionTest, Inverse) {
    Quaternion q = Quaternion::identity();
    Quaternion inv = q.inverse();

    EXPECT_DOUBLE_EQ(inv.w, 1.0);
    EXPECT_DOUBLE_EQ(inv.x, 0.0);
    EXPECT_DOUBLE_EQ(inv.y, 0.0);
    EXPECT_DOUBLE_EQ(inv.z, 0.0);
}

TEST(QuaternionTest, Multiply) {
    Quaternion q1 = Quaternion::identity();
    Quaternion q2 = Quaternion::identity();

    Quaternion result = q1 * q2;

    EXPECT_DOUBLE_EQ(result.w, 1.0);
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(QuaternionTest, RotateVector) {
    // Rotate X-axis (1,0,0) by 90° around Y-axis
    Vec3 axis(0.0, 1.0, 0.0);
    Quaternion q = Quaternion::from_axis_angle(axis, M_PI / 2);

    Vec3 v(1.0, 0.0, 0.0);
    Vec3 rotated = q.rotate(v);

    // Should result in -Z direction
    EXPECT_NEAR(rotated.x, 0.0, 1e-10);
    EXPECT_NEAR(rotated.y, 0.0, 1e-10);
    EXPECT_NEAR(rotated.z, -1.0, 1e-10);
}
