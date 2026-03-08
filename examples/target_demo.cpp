/**
 * @file target_demo.cpp
 * @brief Demonstration of Target class for guidance testing
 *
 * Shows:
 * 1. Stationary target
 * 2. Moving target (constant velocity)
 * 3. Maneuvering target (with acceleration)
 * 4. Target position prediction
 */

#include "guidance/guidance.h"
#include "guidance/proportional_navigation.h"
#include "ballistics/state_6dof.h"
#include "utils/vec3.h"
#include "utils/quaternion.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

void print_target_info(const Target& target, const std::string& name) {
    std::cout << "\n  " << name << ":\n";
    std::cout << "    Position:    " << target.position << " m\n";
    std::cout << "    Velocity:    " << target.velocity << " m/s\n";
    std::cout << "    Speed:       " << target.velocity.magnitude() << " m/s\n";
    std::cout << "    Acceleration: " << target.acceleration << " m/s²\n";
    std::cout << "    Is moving:   " << (target.is_moving() ? "YES" : "NO") << "\n";
    std::cout << "    Has maneuver:" << (target.has_prediction() ? "YES" : "NO") << "\n";
}

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║              TARGET CLASS DEMONSTRATION                    ║\n";
    std::cout << "║              Ballistic Simulation Library                  ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // ========================================================================
    // DEMO 1: Stationary Target
    // ========================================================================
    std::cout << "╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 1: Stationary Target                                 ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n";

    Target stationary_target(Vec3(5000.0, 3000.0, 1000.0));
    print_target_info(stationary_target, "Ground Building");

    // Predict position (should be same)
    Vec3 predicted = stationary_target.predict_position(5.0);
    std::cout << "\n  Prediction after 5.0s:\n";
    std::cout << "    Position: " << predicted << " m\n";
    std::cout << "    (Unchanged - stationary target)\n";

    // ========================================================================
    // DEMO 2: Moving Target (Constant Velocity)
    // ========================================================================
    std::cout << "\n\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 2: Moving Target (Constant Velocity)               ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n";

    Target moving_target(
        Vec3(0.0, 5000.0, 0.0),      // Initial position
        Vec3(250.0, 0.0, 50.0)        // 250 m/s east, 50 m/s north
    );
    print_target_info(moving_target, "Aircraft (Cruise)");

    // Show predictions at different times
    std::cout << "\n  Position predictions:\n";
    std::cout << "  " << std::left << std::setw(12) << "Time (s)"
              << std::setw(25) << "Position (m)"
              << std::setw(15) << "Distance (m)" << std::endl;
    std::cout << "  " << std::string(52, '-') << std::endl;

    Vec3 initial_pos = moving_target.position;
    for (double t = 0.0; t <= 30.0; t += 5.0) {
        Vec3 pred = moving_target.predict_position(t);
        double dist = (pred - initial_pos).magnitude();
        std::cout << "  " << std::left << std::setw(12) << std::fixed << std::setprecision(1)
                  << t << pred << "  " << std::setw(15) << std::setprecision(2) << dist << std::endl;
    }

    // ========================================================================
    // DEMO 3: Maneuvering Target (With Acceleration)
    // ========================================================================
    std::cout << "\n\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 3: Maneuvering Target (Evasive)                      ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n";

    Target maneuvering_target(
        Vec3(8000.0, 6000.0, 2000.0),  // Initial position
        Vec3(200.0, 50.0, -30.0),       // Initial velocity
        Vec3(0.0, 30.0, 0.0)             // 30 m/s² upward acceleration (evasive pull)
    );
    print_target_info(maneuvering_target, "Aircraft (Evasive Climb)");

    // Show trajectory with maneuver
    std::cout << "\n  Maneuvering trajectory:\n";
    std::cout << "  " << std::left << std::setw(10) << "Time(s)"
              << std::setw(12) << "Altitude(m)"
              << std::setw(12) << "Speed(m/s)"
              << std::setw(25) << "Position" << std::endl;
    std::cout << "  " << std::string(59, '-') << std::endl;

    for (double t = 0.0; t <= 10.0; t += 2.0) {
        Vec3 pos = maneuvering_target.predict_position(t);
        Vec3 vel = maneuvering_target.predict_velocity(t);
        std::cout << "  " << std::left << std::setw(10) << std::fixed << std::setprecision(0)
                  << t
                  << std::setw(12) << std::setprecision(1) << pos.y
                  << std::setw(12) << std::setprecision(1) << vel.magnitude()
                  << pos << std::endl;
    }

    // ========================================================================
    // DEMO 4: Target Types for Different Scenarios
    // ========================================================================
    std::cout << "\n\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 4: Target Type Library                                ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    std::vector<std::pair<std::string, Target>> target_types;

    target_types.push_back({"Static Ground Target", Target(Vec3(5000.0, 0.0, 3000.0))});
    target_types.push_back({"Helicopter (Hovering)", Target(Vec3(2000.0, 500.0, 1000.0), Vec3(0.0, 0.0, 10.0))});
    target_types.push_back({"Fighter (Cruise)", Target(Vec3(0.0, 8000.0, 0.0), Vec3(300.0, 0.0, 0.0))});
    target_types.push_back({"Fighter (Afterburner)", Target(Vec3(0.0, 9000.0, 0.0), Vec3(500.0, 50.0, 0.0), Vec3(10.0, 0.0, 0.0))});
    target_types.push_back({"Bomber (Level Flight)", Target(Vec3(15000.0, 10000.0, 5000.0), Vec3(-250.0, 0.0, 0.0))});
    target_types.push_back({"Ship (Surface)", Target(Vec3(5000.0, 0.0, 20000.0), Vec3(15.0, 0.0, 5.0))});
    target_types.push_back({"Evasive Aircraft", Target(Vec3(10000.0, 7000.0, 0.0), Vec3(400.0, 0.0, 0.0), Vec3(0.0, 40.0, 0.0))});

    std::cout << "  " << std::left << std::setw(25) << "Target Type"
              << std::setw(12) << "Speed(m/s)"
              << std::setw(15) << "Accel(m/s²)"
              << std::setw(20) << "Range(m)" << std::endl;
    std::cout << "  " << std::string(72, '-') << std::endl;

    Vec3 missile_pos(0.0, 5000.0, 0.0);
    for (const auto& tt : target_types) {
        double range = (tt.second.position - missile_pos).magnitude();
        std::cout << "  " << std::left << std::setw(25) << tt.first
                  << std::setw(12) << std::fixed << std::setprecision(1) << tt.second.velocity.magnitude()
                  << std::setw(15) << tt.second.acceleration.magnitude()
                  << std::setw(20) << std::setprecision(0) << range << std::endl;
    }

    // ========================================================================
    // DEMO 5: Target Prediction for Guidance
    // ========================================================================
    std::cout << "\n\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 5: Target Prediction for Intercept Calculation        ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    // Create missile and target
    Vec3 missile_pos2(0.0, 5000.0, 0.0);
    Vec3 missile_vel(600.0, 0.0, 0.0);

    Target intercept_target(
        Vec3(12000.0, 6000.0, 1000.0),
        Vec3(-200.0, 0.0, 0.0),  // Head-on!
        Vec3(0.0, 20.0, 0.0)      // Pulling up
    );

    std::cout << "Engagement Scenario:\n";
    std::cout << "  Missile Position: " << missile_pos2 << " m\n";
    std::cout << "  Missile Velocity: " << missile_vel << " m/s\n";
    std::cout << "  Target Position: " << intercept_target.position << " m\n";
    std::cout << "  Target Velocity: " << intercept_target.velocity << " m/s\n";
    std::cout << "  Target Acceleration: " << intercept_target.acceleration << " m/s²\n";

    double initial_range = (intercept_target.position - missile_pos2).magnitude();
    std::cout << "  Initial Range: " << initial_range << " m\n\n";

    // Predict intercept point
    std::cout << "Target Position Predictions:\n";
    std::cout << "  Time(s) | Predicted Position                              | Range(m)\n";
    std::cout << "  --------|--------------------------------------------------|----------\n";

    for (double t = 0.0; t <= 15.0; t += 2.5) {
        Vec3 pred_pos = intercept_target.predict_position(t);
        double range = (pred_pos - missile_pos2).magnitude();

        std::cout << "  " << std::setw(7) << std::fixed << std::setprecision(1) << t
                  << " | " << pred_pos << " | " << std::setprecision(1) << range << std::endl;
    }

    // ========================================================================
    // DEMO 6: Using Target with Guidance
    // ========================================================================
    std::cout << "\n\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 6: Target Integration with Guidance                  ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    auto pn = std::make_unique<ProportionalNavigation>(3.0);
    pn->set_max_acceleration(400.0);
    pn->set_proximity_threshold(100.0);

    State6DOF missile_state(missile_pos2, missile_vel, Quaternion::identity(), Vec3::zero());

    std::cout << "Testing guidance with different target types:\n\n";

    // Test with stationary target
    Target ground_target(Vec3(5000.0, 0.0, 0.0));
    GuidanceCommand cmd1 = pn->calculate_command(missile_state, ground_target);

    std::cout << "1. Stationary Target:\n";
    std::cout << "   Command: " << cmd1.acceleration_command.magnitude()
              << " m/s² (" << (cmd1.acceleration_command.magnitude() / 9.81) << " G)\n";
    std::cout << "   Status: " << cmd1.status_msg << "\n\n";

    // Test with moving target
    Target air_target(Vec3(8000.0, 6000.0, 0.0), Vec3(200.0, 0.0, 0.0));
    GuidanceCommand cmd2 = pn->calculate_command(missile_state, air_target);

    std::cout << "2. Moving Target:\n";
    std::cout << "   Command: " << cmd2.acceleration_command.magnitude()
              << " m/s² (" << (cmd2.acceleration_command.magnitude() / 9.81) << " G)\n";
    std::cout << "   Status: " << cmd2.status_msg << "\n\n";

    // Test with maneuvering target
    Target evasive_target(Vec3(10000.0, 7000.0, 500.0),
                          Vec3(250.0, 0.0, 0.0),
                          Vec3(0.0, 30.0, 0.0));
    GuidanceCommand cmd3 = pn->calculate_command(missile_state, evasive_target);

    std::cout << "3. Maneuvering Target:\n";
    std::cout << "   Command: " << cmd3.acceleration_command.magnitude()
              << " m/s² (" << (cmd3.acceleration_command.magnitude() / 9.81) << " G)\n";
    std::cout << "   Status: " << cmd3.status_msg << "\n";

    // ========================================================================
    // SUMMARY
    // ========================================================================
    std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  TARGET CLASS SUMMARY                                      ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "The Target struct provides:\n\n";
    std::cout << "  ✓ Position tracking [m]\n";
    std::cout << "  ✓ Velocity tracking [m/s]\n";
    std::cout << "  ✓ Acceleration tracking [m/s²]\n";
    std::cout << "  ✓ Future position prediction (constant acceleration model)\n";
    std::cout << "  ✓ Future velocity prediction\n";
    std::cout << "  ✓ Motion detection (is_moving)\n";
    std::cout << "  ✓ Maneuver detection (has_prediction)\n\n";

    std::cout << "Use cases:\n";
    std::cout << "  • Stationary targets (buildings, bunkers)\n";
    std::cout << "  • Moving targets (aircraft, ships, vehicles)\n";
    std::cout << "  • Maneuvering targets (evasive aircraft)\n";
    std::cout << "  • Intercept calculation with prediction\n\n";

    return 0;
}
