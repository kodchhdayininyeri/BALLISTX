#include <gtest/gtest.h>
#include "utils/vec3.h"

using namespace ballistx;

TEST(Vec3Test, DefaultConstructor) {
    Vec3 v;
    EXPECT_DOUBLE_EQ(v.x, 0.0);
    EXPECT_DOUBLE_EQ(v.y, 0.0);
    EXPECT_DOUBLE_EQ(v.z, 0.0);
}

TEST(Vec3Test, ParameterizedConstructor) {
    Vec3 v(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(v.x, 1.0);
    EXPECT_DOUBLE_EQ(v.y, 2.0);
    EXPECT_DOUBLE_EQ(v.z, 3.0);
}

TEST(Vec3Test, Magnitude) {
    Vec3 v(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(v.magnitude(), 5.0);
}

TEST(Vec3Test, MagnitudeSquared) {
    Vec3 v(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(v.magnitude_squared(), 25.0);
}

TEST(Vec3Test, Normalized) {
    Vec3 v(3.0, 4.0, 0.0);
    Vec3 normalized = v.normalized();

    EXPECT_DOUBLE_EQ(normalized.magnitude(), 1.0);
    EXPECT_DOUBLE_EQ(normalized.x, 3.0 / 5.0);
    EXPECT_DOUBLE_EQ(normalized.y, 4.0 / 5.0);
    EXPECT_DOUBLE_EQ(normalized.z, 0.0);
}

TEST(Vec3Test, DotProduct) {
    Vec3 v1(1.0, 2.0, 3.0);
    Vec3 v2(4.0, 5.0, 6.0);

    EXPECT_DOUBLE_EQ(v1.dot(v2), 32.0);  // 1*4 + 2*5 + 3*6 = 32
}

TEST(Vec3Test, CrossProduct) {
    Vec3 v1(1.0, 0.0, 0.0);
    Vec3 v2(0.0, 1.0, 0.0);

    Vec3 result = v1.cross(v2);

    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 1.0);  // i × j = k
}

TEST(Vec3Test, Addition) {
    Vec3 v1(1.0, 2.0, 3.0);
    Vec3 v2(4.0, 5.0, 6.0);

    Vec3 result = v1 + v2;

    EXPECT_DOUBLE_EQ(result.x, 5.0);
    EXPECT_DOUBLE_EQ(result.y, 7.0);
    EXPECT_DOUBLE_EQ(result.z, 9.0);
}

TEST(Vec3Test, Subtraction) {
    Vec3 v1(4.0, 5.0, 6.0);
    Vec3 v2(1.0, 2.0, 3.0);

    Vec3 result = v1 - v2;

    EXPECT_DOUBLE_EQ(result.x, 3.0);
    EXPECT_DOUBLE_EQ(result.y, 3.0);
    EXPECT_DOUBLE_EQ(result.z, 3.0);
}

TEST(Vec3Test, ScalarMultiplication) {
    Vec3 v(1.0, 2.0, 3.0);
    double scalar = 2.0;

    Vec3 result = v * scalar;

    EXPECT_DOUBLE_EQ(result.x, 2.0);
    EXPECT_DOUBLE_EQ(result.y, 4.0);
    EXPECT_DOUBLE_EQ(result.z, 6.0);
}

TEST(Vec3Test, ScalarDivision) {
    Vec3 v(2.0, 4.0, 6.0);
    double scalar = 2.0;

    Vec3 result = v / scalar;

    EXPECT_DOUBLE_EQ(result.x, 1.0);
    EXPECT_DOUBLE_EQ(result.y, 2.0);
    EXPECT_DOUBLE_EQ(result.z, 3.0);
}

TEST(Vec3Test, UnaryMinus) {
    Vec3 v(1.0, 2.0, 3.0);

    Vec3 result = -v;

    EXPECT_DOUBLE_EQ(result.x, -1.0);
    EXPECT_DOUBLE_EQ(result.y, -2.0);
    EXPECT_DOUBLE_EQ(result.z, -3.0);
}

TEST(Vec3Test, DistanceTo) {
    Vec3 v1(0.0, 0.0, 0.0);
    Vec3 v2(3.0, 4.0, 0.0);

    EXPECT_DOUBLE_EQ(v1.distance_to(v2), 5.0);
}

TEST(Vec3Test, StaticFactories) {
    Vec3 zero = Vec3::zero();
    EXPECT_DOUBLE_EQ(zero.x, 0.0);
    EXPECT_DOUBLE_EQ(zero.y, 0.0);
    EXPECT_DOUBLE_EQ(zero.z, 0.0);

    Vec3 up = Vec3::up();
    EXPECT_DOUBLE_EQ(up.x, 0.0);
    EXPECT_DOUBLE_EQ(up.y, 1.0);
    EXPECT_DOUBLE_EQ(up.z, 0.0);

    Vec3 forward = Vec3::forward();
    EXPECT_DOUBLE_EQ(forward.x, 0.0);
    EXPECT_DOUBLE_EQ(forward.y, 0.0);
    EXPECT_DOUBLE_EQ(forward.z, 1.0);

    Vec3 right = Vec3::right();
    EXPECT_DOUBLE_EQ(right.x, 1.0);
    EXPECT_DOUBLE_EQ(right.y, 0.0);
    EXPECT_DOUBLE_EQ(right.z, 0.0);
}
