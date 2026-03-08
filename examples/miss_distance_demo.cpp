/**
 * @file miss_distance_demo.cpp
 * @brief Demonstration of Miss Distance / CPA (Closest Point of Approach) calculation
 *
 * Shows:
 * 1. CPA calculation for static prediction
 * 2. CPA with target acceleration
 * 3. Real-time CPA tracking during simulation
 * 4. Guidance quality evaluation based on miss distance
 * 5. Trajectory analysis
 */

#include "guidance/miss_distance.h"
#include "guidance/proportional_navigation.h"
#include "guidance/guidance.h"
#include "ballistics/state_6dof.h"
#include "utils/vec3.h"
#include "utils/quaternion.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

// Simple simulation for demo
struct SimPoint {
    double time;
    Vec3 missile_pos;
    Vec3 target_pos;
};

/**
 * @brief Run missile simulation and collect trajectory
 */
std::vector<SimPoint> run_simulation(const State6DOF& init_state, const Target& target,
                                    ProportionalNavigation* guidance, double max_time = 15.0) {
    std::vector<SimPoint> trajectory;

    State6DOF state = init_state;
    Target tgt = target;
    double dt = 0.01;
    double t = 0.0;

    while (t < max_time) {
        SimPoint pt;
        pt.time = t;
        pt.missile_pos = state.get_position();
        pt.target_pos = tgt.position;

        trajectory.push_back(pt);

        // Get guidance
        GuidanceCommand cmd = guidance->calculate_command(state, tgt);

        if (cmd.detonation_command || !cmd.is_valid) {
            break;
        }

        // Integrate (simple Euler, no gravity for demo)
        Vec3 accel = cmd.acceleration_command;
        Vec3 vel = state.get_velocity() + accel * dt;
        Vec3 pos = state.get_position() + vel * dt;

        state.set_velocity(vel);
        state.set_position(pos);

        tgt.position = tgt.position + tgt.velocity * dt;

        t += dt;
    }

    return trajectory;
}

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║          MISS DISTANCE / CPA DEMONSTRATION                   ║\n";
    std::cout << "║              Closest Point of Approach Calculation            ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // ========================================================================
    // DEMO 1: Basic CPA Calculation
    // ========================================================================
    std::cout << "╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 1: Basic CPA Calculation                              ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    State6DOF missile_state(Vec3(0.0, 5000.0, 0.0), Vec3(400.0, 0.0, 0.0),
                           Quaternion::identity(), Vec3::zero());
    Target target(Vec3(8000.0, 5200.0, 200.0), Vec3(200.0, 0.0, 0.0));

    std::cout << "Engagement Geometry:\n";
    std::cout << "  Missile: pos=" << missile_state.get_position()
              << ", vel=" << missile_state.get_velocity() << " m/s\n";
    std::cout << "  Target:  pos=" << target.position
              << ", vel=" << target.velocity << " m/s\n";
    std::cout << "  Initial range: " << (target.position - missile_state.get_position()).magnitude() << " m\n\n";

    // Calculate CPA
    auto cpa_result = MissDistance::calculate_cpa(missile_state, target);

    std::cout << "CPA Prediction (Constant Velocity Model):\n";
    cpa_result.print();

    // Quality evaluation
    std::cout << "Guidance Quality: " << MissDistance::get_quality_description(cpa_result.miss_distance) << "\n";
    std::cout << "Quality Score: " << MissDistance::evaluate_quality(cpa_result.miss_distance) << "/100\n";

    // ========================================================================
    // DEMO 2: CPA with Maneuvering Target
    // ========================================================================
    std::cout << "\n\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 2: CPA with Maneuvering Target                        ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    Target evasive_target(Vec3(10000.0, 7000.0, 500.0),
                         Vec3(250.0, 0.0, 0.0),
                         Vec3(0.0, 40.0, 0.0));  // Pulling up

    std::cout << "Evasive Target:\n";
    std::cout << "  Position: " << evasive_target.position << " m\n";
    std::cout << "  Velocity: " << evasive_target.velocity << " m/s\n";
    std::cout << "  Acceleration: " << evasive_target.acceleration << " m/s²\n\n";

    // Compare constant velocity vs accelerated models
    auto cpa_const = MissDistance::calculate_cpa(missile_state, evasive_target);
    auto cpa_accel = MissDistance::calculate_cpa_with_accel(missile_state, evasive_target);

    std::cout << "CPA Comparison:\n";
    std::cout << "  " << std::left << std::setw(25) << "Model"
              << std::setw(15) << "Miss Dist(m)" << std::setw(15) << "Time CPA(s)" << std::endl;
    std::cout << "  " << std::string(55, '-') << std::endl;
    std::cout << "  " << std::left << std::setw(25) << "Constant Velocity"
              << std::setw(15) << std::fixed << std::setprecision(2) << cpa_const.miss_distance
              << std::setw(15) << cpa_const.time_to_cpa << std::endl;
    std::cout << "  " << std::left << std::setw(25) << "With Acceleration"
              << std::setw(15) << std::setprecision(2) << cpa_accel.miss_distance
              << std::setw(15) << cpa_accel.time_to_cpa << std::endl;

    // ========================================================================
    // DEMO 3: Real-time CPA Tracking During Simulation
    // ========================================================================
    std::cout << "\n\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 3: Real-time CPA Tracking During Simulation             ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    auto pn = std::make_unique<ProportionalNavigation>(3.0);
    pn->set_max_acceleration(400.0);
    pn->set_proximity_threshold(50.0);

    State6DOF init_state(Vec3(0.0, 5000.0, 0.0), Vec3(500.0, 0.0, 0.0),
                        Quaternion::identity(), Vec3::zero());
    Target sim_target(Vec3(6000.0, 5300.0, 300.0), Vec3(200.0, 0.0, 0.0));

    MissDistanceTracker tracker;

    // Run simulation with tracking
    double dt = 0.01;
    State6DOF state = init_state;
    Target tgt = sim_target;
    double t = 0.0;

    std::cout << "Time(s) | Range(m) | MinDist(m) | Quality      | Status\n";
    std::cout << "--------|---------|------------|-------------|----------\n";

    while (t < 10.0) {
        // Update tracker
        tracker.update(state.get_position(), tgt.position, t);

        // Get guidance
        GuidanceCommand cmd = pn->calculate_command(state, tgt);

        if (cmd.detonation_command) {
            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(7) << t << " | "
                      << std::setw(7) << (tgt.position - state.get_position()).magnitude() << " | "
                      << std::setw(10) << tracker.get_min_distance() << " | "
                      << std::setw(11) << MissDistance::get_quality_description(tracker.get_min_distance()) << " | HIT\n";
            break;
        }

        if (!cmd.is_valid) {
            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(7) << t << " | "
                      << std::setw(7) << (tgt.position - state.get_position()).magnitude() << " | "
                      << std::setw(10) << tracker.get_min_distance() << " | "
                      << std::setw(11) << "GUIDANCE FAIL" << " | FAIL\n";
            break;
        }

        // Print every 0.5 seconds
        if (std::fmod(t, 0.5) < dt) {
            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(7) << t << " | "
                      << std::setw(7) << (tgt.position - state.get_position()).magnitude() << " | "
                      << std::setw(10) << tracker.get_min_distance() << " | "
                      << std::setw(11) << MissDistance::get_quality_description(tracker.get_min_distance()) << " | TRACKING\n";
        }

        // Integrate
        Vec3 accel = cmd.acceleration_command;
        Vec3 vel = state.get_velocity() + accel * dt;
        Vec3 pos = state.get_position() + vel * dt;

        state.set_velocity(vel);
        state.set_position(pos);
        tgt.position = tgt.position + tgt.velocity * dt;

        t += dt;
    }

    // Final result
    auto final_cpa = tracker.get_result();
    std::cout << "--------|---------|------------|-------------|----------\n";
    std::cout << "FINAL:  |         | " << final_cpa.miss_distance << " | "
              << MissDistance::get_quality_description(final_cpa.miss_distance) << " | ";

    if (final_cpa.intercepted) {
        std::cout << "INTERCEPT\n";
    } else {
        std::cout << "MISS\n";
    }

    // ========================================================================
    // DEMO 4: Post-Flight Trajectory Analysis
    // ========================================================================
    std::cout << "\n\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 4: Post-Flight Trajectory Analysis                    ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    // Run full simulation and collect trajectory
    auto trajectory = run_simulation(init_state, sim_target, pn.get());

    // Convert to tuple format for analysis
    std::vector<std::tuple<double, Vec3, Vec3>> trajectory_points;
    trajectory_points.reserve(trajectory.size());
    for (const auto& pt : trajectory) {
        trajectory_points.push_back(std::make_tuple(pt.time, pt.missile_pos, pt.target_pos));
    }

    // Analyze trajectory
    auto trajectory_cpa = MissDistance::analyze_trajectory(trajectory_points);

    std::cout << "Trajectory Statistics:\n";
    std::cout << "  Total flight time: " << trajectory.back().time << " s\n";
    std::cout << "  Initial range: " << (sim_target.position - init_state.get_position()).magnitude() << " m\n";
    std::cout << "  Final range: " << (trajectory.back().target_pos - trajectory.back().missile_pos).magnitude() << " m\n";
    std::cout << "\n";

    trajectory_cpa.print();

    std::cout << "Guidance Performance:\n";
    std::cout << "  Miss Distance: " << trajectory_cpa.miss_distance << " m\n";
    std::cout << "  Time to CPA: " << trajectory_cpa.time_to_cpa << " s\n";
    std::cout << "  Quality Score: " << MissDistance::evaluate_quality(trajectory_cpa.miss_distance) << "/100\n";
    std::cout << "  Rating: " << MissDistance::get_quality_description(trajectory_cpa.miss_distance) << "\n";

    // ========================================================================
    // DEMO 5: Guidance Quality Comparison
    // ========================================================================
    std::cout << "\n\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 5: Guidance Quality Comparison                        ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    struct TestCase {
        std::string name;
        double nav_gain;
        double missile_speed;
    };

    std::vector<TestCase> tests = {
        {"PN (N=2)", 2.0, 500.0},
        {"PN (N=3)", 3.0, 500.0},
        {"PN (N=5)", 5.0, 500.0},
        {"High Speed", 3.0, 800.0}
    };

    std::cout << std::left << std::setw(15) << "Guidance"
              << std::setw(12) << "Miss(m)"
              << std::setw(10) << "TimeCPA(s)"
              << std::setw(10) << "Quality"
              << std::setw(15) << "Rating" << std::endl;
    std::cout << std::string(62, '-') << std::endl;

    for (const auto& test : tests) {
        auto pn_test = std::make_unique<ProportionalNavigation>(test.nav_gain);
        pn_test->set_max_acceleration(400.0);

        State6DOF test_state(Vec3(0.0, 5000.0, 0.0), Vec3(test.missile_speed, 0.0, 0.0),
                             Quaternion::identity(), Vec3::zero());

        auto traj = run_simulation(test_state, sim_target, pn_test.get());

        std::vector<std::tuple<double, Vec3, Vec3>> points;
        points.reserve(traj.size());
        for (const auto& pt : traj) {
            points.push_back(std::make_tuple(pt.time, pt.missile_pos, pt.target_pos));
        }

        auto cpa = MissDistance::analyze_trajectory(points);

        std::cout << std::left << std::setw(15) << test.name
                  << std::setw(12) << std::fixed << std::setprecision(1) << cpa.miss_distance
                  << std::setw(10) << std::setprecision(2) << cpa.time_to_cpa
                  << std::setw(10) << MissDistance::evaluate_quality(cpa.miss_distance)
                  << std::setw(15) << MissDistance::get_quality_description(cpa.miss_distance) << std::endl;
    }

    // ========================================================================
    // SUMMARY
    // ========================================================================
    std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  MISS DISTANCE / CPA SUMMARY                                ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "CPA (Closest Point of Approach) provides:\n\n";
    std::cout << "  ✓ Miss distance: Minimum separation achieved [m]\n";
    std::cout << "  ✓ Time to CPA: When minimum occurs [s]\n";
    std::cout << "  ✓ CPA positions: Missile and target locations at CPA\n";
    std::cout << "  ✓ Closing velocity: Relative speed at CPA [m/s]\n";
    std::cout << "  ✓ Intercept detection: Direct hit or proximity fuse\n\n";

    std::cout << "Calculation Methods:\n";
    std::cout << "  1. Analytic: Constant velocity model (fast, real-time)\n";
    std::cout << "  2. Numerical: With acceleration (more accurate)\n";
    std::cout << "  3. Trajectory: Post-flight analysis (exact)\n\n";

    std::cout << "Quality Rating Scale:\n";
    std::cout << "  90-100: PERFECT - Direct hit\n";
    std::cout << "  80-89:  EXCELLENT - Proximity fuse intercept\n";
    std::cout << "  60-79:  GOOD - Well guided\n";
    std::cout << "  40-59:  FAIR - Acceptable\n";
    std::cout << "  20-39:  POOR - Marginal\n";
    std::cout << "   0-19:  FAIL - Complete miss\n\n";

    return 0;
}
