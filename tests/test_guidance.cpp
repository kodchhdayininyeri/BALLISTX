#include <gtest/gtest.h>
#include "guidance/guidance.h"
#include "guidance/proportional_navigation.h"
#include "guidance/miss_distance.h"
#include "ballistics/state_6dof.h"
#include "utils/quaternion.h"
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

TEST(GuidanceTest, TargetDefaultConstructor) {
    Target t;
    EXPECT_DOUBLE_EQ(t.position.x, 0.0);
    EXPECT_DOUBLE_EQ(t.position.y, 0.0);
    EXPECT_DOUBLE_EQ(t.position.z, 0.0);
    EXPECT_DOUBLE_EQ(t.velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(t.velocity.y, 0.0);
    EXPECT_DOUBLE_EQ(t.velocity.z, 0.0);
}

TEST(GuidanceTest, TargetParameterizedConstructor) {
    Vec3 pos(100.0, 200.0, 300.0);
    Vec3 vel(10.0, 20.0, 30.0);

    Target t(pos, vel);

    EXPECT_DOUBLE_EQ(t.position.x, 100.0);
    EXPECT_DOUBLE_EQ(t.velocity.x, 10.0);
}

TEST(GuidanceTest, TargetIsMoving) {
    Target stationary(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
    EXPECT_FALSE(stationary.is_moving());

    Target moving(Vec3(0.0, 0.0, 0.0), Vec3(10.0, 0.0, 0.0));
    EXPECT_TRUE(moving.is_moving());
}

TEST(GuidanceTest, TargetHasPrediction) {
    Target no_accel(Vec3(0.0, 0.0, 0.0), Vec3(10.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
    EXPECT_FALSE(no_accel.has_prediction());

    Target with_accel(Vec3(0.0, 0.0, 0.0), Vec3(10.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    EXPECT_TRUE(with_accel.has_prediction());
}

TEST(GuidanceTest, TargetPredictPosition) {
    Vec3 pos(0.0, 0.0, 0.0);
    Vec3 vel(100.0, 0.0, 0.0);
    Vec3 acc(10.0, 0.0, 0.0);

    Target t(pos, vel, acc);

    Vec3 pred = t.predict_position(2.0);

    // p = p0 + v*t + 0.5*a*t²
    // p = 0 + 100*2 + 0.5*10*4 = 220
    EXPECT_DOUBLE_EQ(pred.x, 220.0);
}

TEST(GuidanceTest, ProportionalNavigationConstructor) {
    ProportionalNavigation pn(3.0);
    EXPECT_EQ(pn.get_name(), "Pure Proportional Navigation (PPN)");
}

TEST(GuidanceTest, GuidanceCommandDefaultConstructor) {
    GuidanceCommand cmd;
    EXPECT_DOUBLE_EQ(cmd.acceleration_command.x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.acceleration_command.y, 0.0);
    EXPECT_DOUBLE_EQ(cmd.acceleration_command.z, 0.0);
    EXPECT_FALSE(cmd.detonation_command);
    EXPECT_TRUE(cmd.is_valid);
}

TEST(GuidanceTest, SimpleGuidanceCalculation) {
    ProportionalNavigation pn(3.0);

    Vec3 missile_pos(0.0, 5000.0, 0.0);
    Vec3 missile_vel(500.0, 0.0, 0.0);
    Vec3 target_pos(3000.0, 5000.0, 0.0);
    Vec3 target_vel(200.0, 0.0, 0.0);

    State6DOF state(missile_pos, missile_vel, Quaternion::identity(), Vec3(0.0, 0.0, 0.0));
    Target target(target_pos, target_vel);

    GuidanceCommand cmd = pn.calculate_command(state, target);

    EXPECT_TRUE(cmd.is_valid);
}

TEST(GuidanceTest, MissDistanceCalculation) {
    Vec3 missile_pos(0.0, 5000.0, 0.0);
    Vec3 missile_vel(400.0, 0.0, 0.0);
    Vec3 target_pos(8000.0, 5200.0, 200.0);
    Vec3 target_vel(200.0, 0.0, 0.0);

    State6DOF state(missile_pos, missile_vel, Quaternion::identity(), Vec3(0.0, 0.0, 0.0));
    Target target(target_pos, target_vel);

    auto cpa = MissDistance::calculate_cpa(state, target);

    EXPECT_GT(cpa.miss_distance, 0.0);
    EXPECT_GE(cpa.time_to_cpa, 0.0);
}

TEST(GuidanceTest, MissDistanceQualityEvaluation) {
    EXPECT_EQ(MissDistance::evaluate_quality(0.5), 100);   // Direct hit (< 1.0)
    EXPECT_EQ(MissDistance::evaluate_quality(4.9), 95);    // Proximity fuse (< 5.0)
    EXPECT_EQ(MissDistance::evaluate_quality(5.0), 85);    // Very close (< 10.0) - boundary value
    EXPECT_EQ(MissDistance::evaluate_quality(9.9), 85);    // Very close
    EXPECT_EQ(MissDistance::evaluate_quality(10.0), 70);   // Good (< 25.0) - boundary value
    EXPECT_EQ(MissDistance::evaluate_quality(24.9), 70);   // Good
    EXPECT_EQ(MissDistance::evaluate_quality(25.0), 50);   // Fair (< 50.0) - boundary value
    EXPECT_EQ(MissDistance::evaluate_quality(49.9), 50);   // Fair
    EXPECT_EQ(MissDistance::evaluate_quality(50.0), 30);   // Poor (< 100.0) - boundary value
    EXPECT_EQ(MissDistance::evaluate_quality(99.9), 30);   // Poor
    EXPECT_EQ(MissDistance::evaluate_quality(100.0), 10);  // Very poor (< 500.0) - boundary value
    EXPECT_EQ(MissDistance::evaluate_quality(499.9), 10);  // Very poor
    EXPECT_EQ(MissDistance::evaluate_quality(500.0), 0);   // Fail (>= 500.0) - boundary value
    EXPECT_EQ(MissDistance::evaluate_quality(1000.0), 0);  // Fail
}

TEST(GuidanceTest, MissDistanceTracker) {
    MissDistanceTracker tracker;

    tracker.update(Vec3(0.0, 0.0, 0.0), Vec3(100.0, 0.0, 0.0), 0.0);
    tracker.update(Vec3(10.0, 0.0, 0.0), Vec3(50.0, 0.0, 0.0), 1.0);
    tracker.update(Vec3(20.0, 0.0, 0.0), Vec3(30.0, 0.0, 0.0), 2.0);

    // Check that minimum is correctly tracked
    // Step 0: |(100,0,0) - (0,0,0)| = 100
    // Step 1: |(50,0,0) - (10,0,0)| = 40
    // Step 2: |(30,0,0) - (20,0,0)| = 10
    EXPECT_DOUBLE_EQ(tracker.get_min_distance(), 10.0);

    auto result = tracker.get_result();
    EXPECT_DOUBLE_EQ(result.miss_distance, 10.0);
}

TEST(GuidanceTest, MissDistanceTrackerReset) {
    MissDistanceTracker tracker;

    tracker.update(Vec3(0.0, 0.0, 0.0), Vec3(100.0, 0.0, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(tracker.get_min_distance(), 100.0);

    tracker.reset();
    EXPECT_DOUBLE_EQ(tracker.get_min_distance(), std::numeric_limits<double>::max());
}
