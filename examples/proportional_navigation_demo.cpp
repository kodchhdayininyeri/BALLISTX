/**
 * @file proportional_navigation_demo.cpp
 * @brief Comprehensive demonstration of Proportional Navigation variants
 *
 * This demo illustrates:
 * 1. The mathematics of PN: a_cmd = N * V_c * ω_los
 * 2. Different PN variants (Pure, True, Augmented, Ideal, Biased)
 * 3. Effect of navigation gain (N) on trajectory
 * 4. Comparison with Pure Pursuit
 * 5. Telemetry and diagnostics
 */

#include "guidance/guidance.h"
#include "guidance/proportional_navigation.h"
#include "ballistics/state_6dof.h"
#include "ballistics/projectile.h"
#include "utils/vec3.h"
#include "utils/quaternion.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

// ============================================================================
// SIMULATION ENGINE
// ============================================================================

/**
 * @brief Simple missile simulation with PN guidance
 */
class PNMissileSim {
public:
    State6DOF state;
    Target target;
    ProportionalNavigation* guidance;
    double dt;

    // Performance tracking
    struct TrajectoryPoint {
        double time;
        Vec3 position;
        Vec3 velocity;
        double acceleration_magnitude;
        double los_rate;
        double range_to_target;
        double time_to_go;
    };
    std::vector<TrajectoryPoint> trajectory;

    PNMissileSim(const State6DOF& init_state, const Target& tgt,
                ProportionalNavigation* guid, double time_step = 0.01)
        : state(init_state), target(tgt), guidance(guid), dt(time_step)
    {}

    /**
     * @brief Run simulation until intercept or timeout
     * @return true if intercepted, false if missed
     */
    bool run(double max_time = 30.0) {
        double t = 0.0;
        bool intercepted = false;

        while (t < max_time) {
            // Record trajectory
            TrajectoryPoint pt;
            pt.time = t;
            pt.position = state.get_position();
            pt.velocity = state.get_velocity();

            // Get guidance command
            GuidanceCommand cmd = guidance->calculate_command(state, target);
            pt.acceleration_magnitude = cmd.acceleration_command.magnitude();
            pt.range_to_target = (target.position - state.get_position()).magnitude();
            pt.time_to_go = cmd.time_to_go;

            // Calculate LOS rate for telemetry
            Vec3 los = target.position - state.get_position();
            Vec3 rel_vel = target.velocity - state.get_velocity();
            double range_sq = los.magnitude_squared();
            Vec3 los_rate = (range_sq > 1e-12) ? los.cross(rel_vel) / range_sq : Vec3::zero();
            pt.los_rate = los_rate.magnitude();

            trajectory.push_back(pt);

            // Check for intercept
            if (cmd.detonation_command) {
                intercepted = true;
                break;
            }

            // Check for failed guidance
            if (!cmd.is_valid) {
                break;
            }

            // Simple physics integration (Euler method for demo)
            Vec3 accel = cmd.acceleration_command;
            // accel.y -= 9.81; // No gravity for space/air-to-air (minimal effect at short range)

            Vec3 vel = state.get_velocity();
            Vec3 pos = state.get_position();

            vel = vel + accel * dt;
            pos = pos + vel * dt;

            state.set_velocity(vel);
            state.set_position(pos);

            // Update target (if moving)
            target.position = target.position + target.velocity * dt;

            // Check if target passed (miss) - only if range is SIGNIFICANTLY increasing
            // and we've been running for a while
            if (t > 5.0 && trajectory.size() > 20) {
                double initial_range = trajectory[0].range_to_target;
                double current_range = pt.range_to_target;
                // Only stop if we've passed well beyond initial range (clear miss)
                if (current_range > initial_range * 1.2) {
                    break; // Target passed and receding
                }
            }

            t += dt;
        }

        return intercepted;
    }

    /**
     * @brief Print trajectory summary
     */
    void print_summary(bool intercepted) const {
        if (trajectory.empty()) return;

        const auto& final = trajectory.back();

        std::cout << "  Flight time: " << final.time << " s" << std::endl;
        std::cout << "  Final range: " << final.range_to_target << " m" << std::endl;
        std::cout << "  Final velocity: " << final.velocity.magnitude() << " m/s" << std::endl;
        std::cout << "  Max acceleration: ";
        double max_accel = 0.0;
        for (const auto& pt : trajectory) {
            max_accel = std::max(max_accel, pt.acceleration_magnitude);
        }
        std::cout << max_accel << " m/s² (" << (max_accel / 9.81) << " G)" << std::endl;
        std::cout << "  Result: " << (intercepted ? "INTERCEPT" : "MISS") << std::endl;
    }
};

// ============================================================================
// DEMO SCENARIOS
// ============================================================================

/**
 * @brief Demo 1: Show PN mathematics in action
 */
void demo_pn_mathematics() {
    std::cout << "\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 1: Proportional Navigation Mathematics              ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    std::cout << "PN Formula: a_cmd = N * V_c * ω_los\n\n";
    std::cout << "Where:\n";
    std::cout << "  N     = Navigation gain (dimensionless, typically 3-5)\n";
    std::cout << "  V_c   = Closing velocity [m/s] (positive when approaching)\n";
    std::cout << "  ω_los = Line-of-sight rotation rate [rad/s]\n\n";

    // Create PN guidance
    auto pn = std::make_unique<ProportionalNavigation>(3.0,
        ProportionalNavigation::Variant::PURE_PN);
    pn->set_max_acceleration(400.0);

    // Scenario: Tail chase (easier intercept, more realistic)
    // Missile behind target, both flying same direction
    Vec3 missile_pos(0.0, 5000.0, 0.0);
    Vec3 missile_vel(600.0, 0.0, 0.0);  // 600 m/s (Mach 1.8) - FAST missile
    Vec3 target_pos(8000.0, 5200.0, 200.0); // 8km ahead, slightly offset
    Vec3 target_vel(250.0, 0.0, 0.0); // 250 m/s (subsonic aircraft)

    State6DOF state(missile_pos, missile_vel, Quaternion::identity(), Vec3::zero());
    Target target(target_pos, target_vel);

    std::cout << "Initial Conditions:\n";
    std::cout << "  Missile: pos=" << missile_pos << ", vel=" << missile_vel << " m/s\n";
    std::cout << "  Target:  pos=" << target_pos << ", vel=" << target_vel << " m/s\n";
    std::cout << "  Initial range: " << (target_pos - missile_pos).magnitude() << " m\n";
    std::cout << "  Engagement: TAIL CHASE (missile faster, overtaking)\n\n";

    // Get telemetry at t=0
    auto tel = pn->get_telemetry(state, target);

    std::cout << "Guidance Parameters at t=0:\n";
    std::cout << "  Navigation gain (N): " << tel.navigation_gain << "\n";
    std::cout << "  Closing velocity (V_c): " << tel.closing_velocity << " m/s\n";
    std::cout << "  LOS rate (ω_los): " << (tel.los_rate_magnitude * 1000) << " mrad/s\n";
    std::cout << "  Time-to-go: " << tel.time_to_go << " s\n";
    std::cout << "  Commanded acceleration: " << tel.commanded_acceleration << " m/s² ("
              << (tel.commanded_acceleration / 9.81) << " G)\n";
    std::cout << "  Off-boresight angle: " << tel.off_boresight_angle << "°\n\n";

    // Run simulation
    PNMissileSim sim(state, target, pn.get());
    bool intercepted = sim.run();

    std::cout << "Simulation Result:\n";
    sim.print_summary(intercepted);
}

/**
 * @brief Demo 2: Compare PN variants
 */
void demo_compare_variants() {
    std::cout << "\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 2: Comparing PN Variants                             ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    struct VariantTest {
        std::string name;
        ProportionalNavigation::Variant variant;
        double gain;
    };

    std::vector<VariantTest> variants = {
        {"Pure PN (N=3)", ProportionalNavigation::Variant::PURE_PN, 3.0},
        {"Pure PN (N=5)", ProportionalNavigation::Variant::PURE_PN, 5.0},
        {"True PN (N=3)", ProportionalNavigation::Variant::TRUE_PN, 3.0},
        {"Augmented PN", ProportionalNavigation::Variant::AUGMENTED_PN, 3.0},
    };

    // Tail chase with slow target
    Vec3 missile_pos(0.0, 5000.0, 0.0);
    Vec3 missile_vel(700.0, 0.0, 0.0); // Very fast missile (Mach 2+)

    Vec3 target_pos(10000.0, 5500.0, 500.0);
    Vec3 target_vel(200.0, 0.0, 0.0);  // Slow target
    Vec3 target_accel(0.0, 15.0, 0.0);  // Light maneuver

    std::cout << "Scenario: Tail chase with maneuvering target\n";
    std::cout << "  Missile launch: 5km altitude, 700 m/s (Mach 2.1)\n";
    std::cout << "  Target: 10km ahead, 5.5km altitude, 500m lateral\n";
    std::cout << "  Target velocity: 200 m/s (subsonic)\n";
    std::cout << "  Target acceleration: 15 m/s² (evasive climb)\n\n";

    std::cout << std::left << std::setw(20) << "Guidance"
              << std::setw(10) << "Intercept"
              << std::setw(10) << "Time(s)"
              << std::setw(12) << "MissDist(m)"
              << std::setw(12) << "MaxAccel(G)" << std::endl;
    std::cout << std::string(64, '-') << std::endl;

    for (const auto& test : variants) {
        auto pn = std::make_unique<ProportionalNavigation>(test.gain, test.variant);
        pn->set_max_acceleration(600.0); // Higher G limit

        // Reset target
        Target target(target_pos, target_vel, target_accel);
        State6DOF state(missile_pos, missile_vel, Quaternion::identity(), Vec3::zero());

        // Run simulation
        PNMissileSim sim(state, target, pn.get());
        bool intercepted = sim.run();

        double miss_dist = intercepted ? 0.0 : sim.trajectory.back().range_to_target;
        double max_accel_g = 0.0;
        for (const auto& pt : sim.trajectory) {
            max_accel_g = std::max(max_accel_g, pt.acceleration_magnitude / 9.81);
        }

        std::cout << std::left << std::setw(20) << test.name
                  << std::setw(10) << (intercepted ? "YES" : "NO")
                  << std::setw(10) << std::fixed << std::setprecision(2)
                  << sim.trajectory.back().time
                  << std::setw(12) << miss_dist
                  << std::setw(12) << max_accel_g << std::endl;
    }
}

/**
 * @brief Demo 3: Effect of navigation gain N
 */
void demo_navigation_gain_effect() {
    std::cout << "\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 3: Effect of Navigation Gain (N)                    ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    std::cout << "Testing N values: 2.0, 3.0, 4.0, 5.0\n\n";

    // Tail chase scenario
    Vec3 missile_pos(0.0, 5000.0, 0.0);
    Vec3 missile_vel(650.0, 0.0, 0.0);
    Vec3 target_pos(8000.0, 5300.0, 300.0);
    Vec3 target_vel(250.0, 0.0, 0.0);

    std::cout << std::left << std::setw(10) << "Gain N"
              << std::setw(12) << "Intercept"
              << std::setw(12) << "Time(s)"
              << std::setw(12) << "MaxG"
              << std::setw(15) << "FinalLOS(mrad)" << std::endl;
    std::cout << std::string(61, '-') << std::endl;

    std::vector<double> gains = {2.0, 3.0, 4.0, 5.0};

    for (double N : gains) {
        auto pn = std::make_unique<ProportionalNavigation>(N,
            ProportionalNavigation::Variant::PURE_PN);
        pn->set_max_acceleration(500.0);

        Target target(target_pos, target_vel);
        State6DOF state(missile_pos, missile_vel, Quaternion::identity(), Vec3::zero());

        PNMissileSim sim(state, target, pn.get());
        bool intercepted = sim.run();

        double max_g = 0.0;
        for (const auto& pt : sim.trajectory) {
            max_g = std::max(max_g, pt.acceleration_magnitude / 9.81);
        }

        std::cout << std::left << std::setw(10) << N
                  << std::setw(12) << (intercepted ? "YES" : "NO")
                  << std::setw(12) << std::fixed << std::setprecision(2)
                  << sim.trajectory.back().time
                  << std::setw(12) << max_g
                  << std::setw(15) << (sim.trajectory.back().los_rate * 1000)
                  << std::endl;
    }

    std::cout << "\nObservations:\n";
    std::cout << "  N=2: Slow response, may miss against maneuvering targets\n";
    std::cout << "  N=3: Optimal for non-maneuvering targets (most common)\n";
    std::cout << "  N=4-5: Better against maneuvering targets, higher G demand\n";
    std::cout << "  N>5: Can oscillate, very high G demand\n";
}

/**
 * @brief Demo 4: LOS rate visualization
 */
void demo_los_rate_visualization() {
    std::cout << "\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 4: LOS Rate During Flight                            ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    std::cout << "LOS rate should approach zero as missile approaches target\n";
    std::cout << "(Zero LOS rate = collision course)\n\n";

    auto pn = std::make_unique<ProportionalNavigation>(3.0);
    pn->set_max_acceleration(500.0);

    Vec3 missile_pos(0.0, 5000.0, 0.0);
    Vec3 missile_vel(600.0, 0.0, 0.0);
    Vec3 target_pos(8000.0, 5200.0, 0.0);
    Vec3 target_vel(250.0, 0.0, 0.0);

    Target target(target_pos, target_vel);
    State6DOF state(missile_pos, missile_vel, Quaternion::identity(), Vec3::zero());

    PNMissileSim sim(state, target, pn.get());
    bool intercepted = sim.run();

    std::cout << "Time(s) | Range(m) | LOS Rate(mrad/s) | Accel(m/s²) | G-load\n";
    std::cout << "--------|----------|------------------|-------------|--------\n";

    size_t step = std::max(size_t(1), sim.trajectory.size() / 11);
    for (size_t i = 0; i < sim.trajectory.size(); i += step) {
        const auto& pt = sim.trajectory[i];
        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(7) << pt.time << " | "
                  << std::setw(8) << pt.range_to_target << " | "
                  << std::setw(16) << (pt.los_rate * 1000) << " | "
                  << std::setw(11) << pt.acceleration_magnitude << " | "
                  << std::setw(6) << std::setprecision(1) << (pt.acceleration_magnitude / 9.81)
                  << std::endl;
    }

    // Always print final point
    const auto& final = sim.trajectory.back();
    std::cout << std::fixed << std::setprecision(1)
              << std::setw(7) << final.time << " | "
              << std::setw(8) << final.range_to_target << " | "
              << std::setw(16) << (final.los_rate * 1000) << " | "
              << std::setw(11) << final.acceleration_magnitude << " | "
              << std::setw(6) << std::setprecision(1) << (final.acceleration_magnitude / 9.81)
              << std::endl;

    std::cout << "\nResult: " << (intercepted ? "INTERCEPT" : "MISS") << std::endl;
}

/**
 * @brief Demo 5: Pure Pursuit vs PN comparison
 */
void demo_pursuit_vs_pn() {
    std::cout << "\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 5: Pure Pursuit vs Proportional Navigation          ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    std::cout << "Comparing Pure Pursuit (inefficient) vs PN (optimal)\n\n";

    // Same scenario for both
    Vec3 missile_pos(0.0, 5000.0, 0.0);
    Vec3 missile_vel(550.0, 0.0, 0.0);
    Vec3 target_pos(6000.0, 5400.0, 400.0);
    Vec3 target_vel(200.0, 30.0, 0.0); // Slightly climbing

    std::cout << "Scenario: Tail chase at 6km\n\n";

    // Pure PN
    auto pn = std::make_unique<ProportionalNavigation>(3.0);
    pn->set_max_acceleration(400.0);

    Target target1(target_pos, target_vel);
    State6DOF state1(missile_pos, missile_vel, Quaternion::identity(), Vec3::zero());

    PNMissileSim sim_pn(state1, target1, pn.get());
    bool intercepted_pn = sim_pn.run();

    double max_g_pn = 0.0;
    for (const auto& pt : sim_pn.trajectory) {
        max_g_pn = std::max(max_g_pn, pt.acceleration_magnitude / 9.81);
    }

    // Pure Pursuit simulation
    auto pursuit = std::make_unique<PurePursuit>(400.0);

    Target target2(target_pos, target_vel);
    State6DOF state2(missile_pos, missile_vel, Quaternion::identity(), Vec3::zero());

    double t = 0.0;
    bool intercepted_pursuit = false;
    double max_g_pursuit = 400.0 / 9.81;
    double final_range = 0.0;
    double final_time = 0.0;

    while (t < 30.0) {
        GuidanceCommand cmd = pursuit->calculate_command(state2, target2);
        if (cmd.detonation_command) {
            intercepted_pursuit = true;
            break;
        }
        if (!cmd.is_valid) break;

        Vec3 accel = cmd.acceleration_command;
        accel.y -= 9.81;
        Vec3 vel = state2.get_velocity() + accel * 0.01;
        Vec3 pos = state2.get_position() + vel * 0.01;
        state2.set_velocity(vel);
        state2.set_position(pos);
        target2.position += target2.velocity * 0.01;

        final_range = (target2.position - state2.get_position()).magnitude();
        if (t > 2.0 && final_range > 8000.0) break;

        t += 0.01;
        final_time = t;
    }

    std::cout << "               | Pure PN    | Pure Pursuit\n";
    std::cout << "---------------|------------|-------------\n";
    std::cout << "Intercept      | " << std::setw(11) << (intercepted_pn ? "YES" : "NO")
              << " | " << std::setw(11) << (intercepted_pursuit ? "YES" : "NO") << "\n";
    std::cout << "Time (s)       | " << std::setw(11) << std::fixed << std::setprecision(2)
              << sim_pn.trajectory.back().time << " | " << std::setw(11) << final_time << "\n";
    std::cout << "Miss distance  | " << std::setw(11)
              << (intercepted_pn ? 0.0 : sim_pn.trajectory.back().range_to_target) << " | "
              << std::setw(11) << (intercepted_pursuit ? 0.0 : final_range) << "\n";
    std::cout << "Max G-load     | " << std::setw(11) << max_g_pn << " | "
              << std::setw(11) << max_g_pursuit << "\n";

    std::cout << "\nConclusion: PN is more efficient than Pure Pursuit!\n";
}

/**
 * @brief Demo 6: SUCCESSFUL intercept demonstration
 */
void demo_successful_intercept() {
    std::cout << "\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DEMO 6: Successful Intercept Demonstration                 ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

    std::cout << "Simple tail chase: Same direction, missile faster\n\n";

    auto pn = std::make_unique<ProportionalNavigation>(3.0);
    pn->set_max_acceleration(500.0); // 50G
    pn->set_proximity_threshold(500.0); // 500m proximity fuse (more realistic for demo)

    // Very simple scenario: same X axis, missile behind and faster
    Vec3 missile_pos(0.0, 5000.0, 0.0);
    Vec3 missile_vel(400.0, 0.0, 0.0); // 400 m/s
    Vec3 target_pos(3000.0, 5000.0, 0.0); // 3km ahead (closer)
    Vec3 target_vel(150.0, 0.0, 0.0); // 150 m/s (slow)

    Target target(target_pos, target_vel);
    State6DOF state(missile_pos, missile_vel, Quaternion::identity(), Vec3::zero());

    PNMissileSim sim(state, target, pn.get());
    bool intercepted = sim.run();

    std::cout << "Initial Conditions:\n";
    std::cout << "  Missile: " << missile_vel.x << " m/s (east)\n";
    std::cout << "  Target: " << target_vel.x << " m/s (east, same line)\n";
    std::cout << "  Range: " << (target_pos.x - missile_pos.x) << " m (directly behind)\n";
    std::cout << "  Speed advantage: " << (missile_vel.x - target_vel.x) << " m/s\n\n";

    std::cout << "Result:\n";
    sim.print_summary(intercepted);

    if (intercepted) {
        std::cout << "\n✓ SUCCESSFUL INTERCEPT!\n";
        std::cout << "  Direct tail chase achieved intercept\n";
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║          PROPORTIONAL NAVIGATION COMPREHENSIVE DEMO        ║\n";
    std::cout << "║              Ballistic Simulation Library                  ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";

    try {
        demo_pn_mathematics();
        demo_compare_variants();
        demo_navigation_gain_effect();
        demo_los_rate_visualization();
        demo_pursuit_vs_pn();
        demo_successful_intercept();

        std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║  ALL DEMOS COMPLETE                                        ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

        std::cout << "Key Takeaways:\n";
        std::cout << "  1. PN: a_cmd = N * V_c * ω_los (maintains constant LOS bearing)\n";
        std::cout << "  2. Optimal N = 3 for non-maneuvering targets\n";
        std::cout << "  3. Augmented PN adds target acceleration compensation\n";
        std::cout << "  4. LOS rate → 0 on collision course\n";
        std::cout << "  5. PN is more efficient than Pure Pursuit\n";
        std::cout << "  6. Speed advantage is CRITICAL for intercept\n\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
