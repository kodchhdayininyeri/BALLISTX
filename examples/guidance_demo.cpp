/**
 * @file guidance_demo.cpp
 * @brief Demonstration of guidance algorithms
 *
 * Shows how to use different guidance laws for missile intercept simulation.
 */

#include "guidance/guidance.h"
#include "guidance/proportional_navigation.h"
#include "ballistics/state_6dof.h"
#include "ballistics/projectile.h"
#include "utils/vec3.h"
#include "utils/integrator.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <algorithm>

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

/**
 * @brief Simple missile simulation with guidance
 */
struct MissileSim {
    State6DOF state;
    Projectile projectile;
    Guidance* guidance;
    Target target;

    double max_lateral_accel;  // m/s²
    double dt;                 // Time step [s]

    void step() {
        // Get guidance command
        GuidanceCommand cmd = guidance->calculate_command(state, target);

        if (!cmd.is_valid) {
            std::cout << "Guidance failed: " << cmd.status_msg << std::endl;
            return;
        }

        // Check for detonation
        if (cmd.detonation_command) {
            std::cout << "=== DETONATION ===" << std::endl;
            std::cout << "Time to go: " << cmd.time_to_go << " s" << std::endl;
            double miss_distance = (target.position - state.get_position()).magnitude();
            std::cout << "Miss distance: " << miss_distance << " m" << std::endl;
            return;
        }

        // Calculate acceleration (simplified - only lateral guidance)
        Vec3 accel = cmd.acceleration_command;

        // Add gravity
        accel.y -= 9.81;

        // Integrate (simple Euler for demo)
        Vec3 vel = state.get_velocity();
        Vec3 pos = state.get_position();

        vel += accel * dt;
        pos += vel * dt;

        state.set_velocity(vel);
        state.set_position(pos);

        // Update target (moving target scenario)
        target.position += target.velocity * dt;
    }
};

/**
 * @brief Run intercept simulation
 */
void run_intercept_demo() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "PROPORTIONAL NAVIGATION INTERCEPT DEMO" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Create missile projectile (AMRAAM-like)
    Projectile missile = Projectile::create(ProjectileType::MISSILE_AIR_TO_AIR);
    std::cout << "Missile: " << missile.get_name() << std::endl;
    std::cout << "  Mass: " << missile.get_mass() << " kg" << std::endl;
    std::cout << "  Diameter: " << (missile.get_diameter() * 1000) << " mm" << std::endl;

    // Create guidance (Proportional Navigation with N=3)
    auto pn_guidance = std::make_unique<ProportionalNavigation>(3.0);
    pn_guidance->set_max_acceleration(300.0); // 30G max lateral
    pn_guidance->set_fov_limit(60.0); // 60 degree seeker FOV
    pn_guidance->set_proximity_threshold(10.0); // 10m proximity fuse

    std::cout << "\nGuidance: " << pn_guidance->get_name() << std::endl;
    std::cout << "  Navigation gain: 3.0" << std::endl;
    std::cout << "  Max lateral accel: 300 m/s² (~30G)" << std::endl;
    std::cout << "  Seeker FOV: 60°" << std::endl;

    // Initial conditions
    Vec3 missile_pos(0.0, 5000.0, 0.0);    // 5km altitude
    Vec3 missile_vel(250.0, 0.0, 0.0);     // 250 m/s (Mach 0.8 launch)
    Vec3 missile_angular_vel(0.0, 0.0, 0.0);

    // Target (aircraft at 10km, moving perpendicular)
    Vec3 target_pos(8000.0, 8000.0, 2000.0); // 8km downrange, 8km altitude
    Vec3 target_vel(-200.0, 0.0, 0.0);      // 200 m/s crossing track
    Vec3 target_accel(0.0, 0.0, 0.0);

    Target target(target_pos, target_vel, target_accel);

    std::cout << "\nInitial Conditions:" << std::endl;
    std::cout << "  Missile pos: " << missile_pos << " m" << std::endl;
    std::cout << "  Missile vel: " << missile_vel << " m/s" << std::endl;
    std::cout << "  Target pos: " << target_pos << " m" << std::endl;
    std::cout << "  Target vel: " << target_vel << " m/s" << std::endl;

    double initial_range = (target_pos - missile_pos).magnitude();
    std::cout << "  Initial range: " << initial_range << " m" << std::endl;

    // Set up simulation
    MissileSim sim;
    sim.state = State6DOF(missile_pos, missile_vel, Quaternion::identity(), missile_angular_vel);
    sim.projectile = missile;
    sim.guidance = pn_guidance.get();
    sim.target = target;
    sim.max_lateral_accel = 300.0;
    sim.dt = 0.01; // 10ms time step

    std::cout << "\n=== TRAJECTORY ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time(s) | Range(m) | Alt(m) | Speed(m/s) | Accel(m/s²) | TimeToGo(s)" << std::endl;
    std::cout << "--------|----------|--------|------------|-------------|-------------" << std::endl;

    // Run simulation
    double t = 0.0;
    const double max_time = 30.0;

    while (t < max_time) {
        // Get guidance command for display
        GuidanceCommand cmd = sim.guidance->calculate_command(sim.state, sim.target);
        double range = (sim.target.position - sim.state.get_position()).magnitude();
        double speed = sim.state.get_velocity().magnitude();

        // Print every 0.5 seconds
        if (std::fmod(t, 0.5) < sim.dt) {
            std::cout << std::setw(7) << t << " | "
                     << std::setw(8) << range << " | "
                     << std::setw(6) << sim.state.y() << " | "
                     << std::setw(10) << speed << " | "
                     << std::setw(11) << cmd.acceleration_command.magnitude() << " | "
                     << std::setw(11) << cmd.time_to_go << std::endl;
        }

        sim.step();
        t += sim.dt;

        // Check for intercept
        if (cmd.detonation_command) {
            std::cout << "\n=== INTERCEPT at t=" << t << " s ===" << std::endl;
            break;
        }

        // Check if target passed
        if (range > initial_range * 1.5) {
            std::cout << "\n=== TARGET PASSED - MISS ===" << std::endl;
            break;
        }
    }
}

/**
 * @brief Compare guidance laws
 */
void compare_guidance_laws() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "COMPARING GUIDANCE LAWS" << std::endl;
    std::cout << "========================================\n" << std::endl;

    struct TestCase {
        std::string name;
        std::unique_ptr<Guidance> guidance;
    };

    std::vector<TestCase> cases;

    // Pure PN
    auto pn = std::make_unique<ProportionalNavigation>(3.0);
    pn->set_max_acceleration(300.0);
    cases.push_back({"Pure PN (N=3)", std::move(pn)});

    // High-gain PN
    auto pn_high = std::make_unique<ProportionalNavigation>(5.0);
    pn_high->set_max_acceleration(300.0);
    cases.push_back({"Pure PN (N=5)", std::move(pn_high)});

    // Pure Pursuit
    auto pursuit = std::make_unique<PurePursuit>(300.0);
    cases.push_back({"Pure Pursuit", std::move(pursuit)});

    // Initial conditions (same for all)
    Vec3 missile_pos(0.0, 5000.0, 0.0);
    Vec3 missile_vel(300.0, 0.0, 0.0);

    Vec3 target_pos(10000.0, 6000.0, 0.0);
    Vec3 target_vel(0.0, 0.0, 0.0); // Stationary target

    std::cout << "Scenario: Head-on intercept vs stationary target" << std::endl;
    std::cout << "  Launch: 5km altitude, 300 m/s" << std::endl;
    std::cout << "  Target: 10km downrange, 6km altitude, stationary" << std::endl;

    std::cout << "\nResults:" << std::endl;
    std::cout << std::left << std::setw(20) << "Guidance"
              << std::setw(12) << "Intercept(s)"
              << std::setw(12) << "FinalRange(m)"
              << std::setw(12) << "MissDist(m)" << std::endl;
    std::cout << std::string(56, '-') << std::endl;

    for (auto& test : cases) {
        // Reset target
        Target target(target_pos, target_vel);

        // Reset state
        State6DOF state(missile_pos, missile_vel, Quaternion::identity(), Vec3::zero());

        // Simulate
        double t = 0.0;
        const double dt = 0.01;
        const double max_time = 40.0;
        bool intercepted = false;
        double miss_distance = 0.0;

        while (t < max_time) {
            GuidanceCommand cmd = test.guidance->calculate_command(state, target);

            if (cmd.detonation_command) {
                intercepted = true;
                miss_distance = (target.position - state.get_position()).magnitude();
                break;
            }

            // Simple integration
            Vec3 accel = cmd.acceleration_command;
            accel.y -= 9.81; // Gravity

            Vec3 vel = state.get_velocity();
            Vec3 pos = state.get_position();

            vel += accel * dt;
            pos += vel * dt;

            state.set_velocity(vel);
            state.set_position(pos);

            target.position += target.velocity * dt;
            t += dt;

            // Check if passed
            double range = (target.position - state.get_position()).magnitude();
            if (range > 15000.0) {
                miss_distance = range;
                break;
            }
        }

        std::cout << std::left << std::setw(20) << test.name
                  << std::setw(12) << std::fixed << std::setprecision(2) << t
                  << std::setw(12) << (target.position - state.get_position()).magnitude()
                  << std::setw(12) << miss_distance << std::endl;
    }
}

/**
 * @brief Demonstrate seeker FOV constraints
 */
void demo_fov_constraint() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "SEEKER FOV CONSTRAINT DEMO" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Create PN guidance with FOV limit
    auto pn = std::make_unique<ProportionalNavigation>(3.0);
    pn->set_fov_limit(30.0); // 30 degree FOV
    pn->set_max_acceleration(500.0);

    // High off-boresight shot
    Vec3 missile_pos(0.0, 5000.0, 0.0);
    Vec3 missile_vel(300.0, 0.0, 0.0); // Flying east

    Vec3 target_pos(2000.0, 6000.0, 4000.0); // 60 degrees off nose
    Vec3 target_vel(0.0, 0.0, 0.0);

    State6DOF state(missile_pos, missile_vel, Quaternion::identity(), Vec3::zero());
    Target target(target_pos, target_vel);

    std::cout << "Scenario: High off-boresight engagement" << std::endl;
    std::cout << "  Missile heading: East (0° azimuth)" << std::endl;
    std::cout << "  Target bearing: ~63° off nose" << std::endl;
    std::cout << "  Seeker FOV limit: 30°" << std::endl;

    bool can_engage = pn->can_engage(state, target);
    std::cout << "\nCan engage: " << (can_engage ? "YES" : "NO") << std::endl;

    if (!can_engage) {
        std::cout << "Target outside seeker field-of-view!" << std::endl;
        std::cout << "\nIncreasing FOV to 70°..." << std::endl;
        pn->set_fov_limit(70.0);
        can_engage = pn->can_engage(state, target);
        std::cout << "Can engage: " << (can_engage ? "YES" : "NO") << std::endl;
    }
}

int main() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════╗\n";
    std::cout << "║   BALLISTX GUIDANCE SYSTEM DEMO         ║\n";
    std::cout << "║   Ballistic Simulation Library          ║\n";
    std::cout << "╚══════════════════════════════════════════╝\n";

    try {
        run_intercept_demo();
        compare_guidance_laws();
        demo_fov_constraint();

        std::cout << "\n=== Demo Complete ===" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
