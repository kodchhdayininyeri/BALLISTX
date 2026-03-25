/**
 * @file ekf_demo.cpp
 * @brief Demonstration of Extended Kalman Filter for radar tracking
 *
 * This example shows how to use the EKF to track a projectile with noisy radar measurements.
 */

#include "sensors/kalman_filter.h"
#include "utils/integrator.h"
#include "utils/vec3.h"
#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

// Process model that matches the true dynamics (gravity + drag)
State6DOF projectile_dynamics(const State6DOF& state, double /*t*/) {
    State6DOF ds;
    ds.set_position(state.get_velocity());

    // Gravity
    Vec3 gravity(0.0, -9.81, 0.0);

    // Simple drag model
    Vec3 vel = state.get_velocity();
    double speed = vel.magnitude();
    double drag_factor = 0.001;  // Simplified drag coefficient
    Vec3 drag = vel * (-drag_factor * speed);

    Vec3 accel = gravity + drag;
    ds.set_velocity(accel);

    // Quaternion and angular velocity (simplified - no rotation for this demo)
    ds.set_orientation(Quaternion::identity());
    ds.set_angular_velocity(Vec3(0.0, 0.0, 0.0));

    return ds;
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║        Extended Kalman Filter - Radar Tracking Demo           ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

    // Simulation parameters
    const double dt = 0.01;
    const double duration = 8.0;
    const size_t num_steps = static_cast<size_t>(duration / dt);

    // True initial state (artillery shell)
    Vec3 true_pos(0.0, 0.0, 0.0);
    Vec3 true_vel(200.0, 100.0, 0.0);
    Quaternion true_quat = Quaternion::identity();
    Vec3 true_ang_vel(0.0, 0.0, 0.0);

    State6DOF true_state(true_pos, true_vel, true_quat, true_ang_vel);

    // Radar noise parameters (typical values for a military radar)
    const double range_std = 15.0;        // 15m range error
    const double azimuth_std = 0.003;     // ~0.17° azimuth error
    const double elevation_std = 0.002;   // ~0.11° elevation error
    const double range_rate_std = 2.0;    // 2 m/s range rate error

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 1.0);

    // Initialize EKF with initial estimate (slightly different from true state)
    State6DOF initial_estimate(
        Vec3(20.0, 10.0, 0.0),        // 20m position error
        Vec3(190.0, 90.0, 0.0),       // 10 m/s velocity error
        Quaternion::identity(),
        Vec3(0.0, 0.0, 0.0)
    );

    ExtendedKalmanFilter ekf(initial_estimate, 30.0);  // 30m initial uncertainty

    // Configure process noise (model uncertainty)
    // Higher values make the filter trust measurements more
    ekf.set_process_noise(0.5, 0.2);

    // Configure measurement noise (radar specifications)
    RadarMeasurement R;
    R.range = range_std;
    R.azimuth = azimuth_std;
    R.elevation = elevation_std;
    R.range_rate = range_rate_std;
    ekf.set_measurement_noise(R);

    std::cout << "Initial State:\n";
    std::cout << "  True Position:  " << true_state.get_position() << " m\n";
    std::cout << "  True Velocity:  " << true_state.get_velocity() << " m/s\n";
    std::cout << "  Est Position:   " << ekf.get_position() << " m\n";
    std::cout << "  Est Velocity:   " << ekf.get_velocity() << " m/s\n";
    std::cout << "  Initial Uncertainty: " << ekf.get_position_uncertainty() << " m\n\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Tracking Results:\n\n";
    std::cout << "Time(s) | True Pos (m)              | Est Pos (m)                | Err (m) | Unc (m)\n";
    std::cout << "--------|---------------------------|----------------------------|---------|--------\n";

    // Tracking loop
    for (size_t step = 0; step <= num_steps; ++step) {
        double t = step * dt;

        Vec3 true_position = true_state.get_position();
        Vec3 true_velocity = true_state.get_velocity();

        // Generate noisy radar measurement
        RadarMeasurement z = RadarMeasurement::from_state(true_state);
        z.range += range_std * noise(gen);
        z.azimuth += azimuth_std * noise(gen);
        z.elevation += elevation_std * noise(gen);
        z.range_rate += range_rate_std * noise(gen);

        // EKF prediction step (with physics model)
        ekf.predict(dt, projectile_dynamics);

        // EKF update step (incorporate measurement)
        ekf.update(z);

        // Get filtered estimate
        Vec3 est_position = ekf.get_position();
        Vec3 est_velocity = ekf.get_velocity();
        double uncertainty = ekf.get_position_uncertainty();

        // Compute error
        double pos_error = true_position.distance_to(est_position);

        // Print every 50 steps
        if (step % 50 == 0 || step == num_steps) {
            std::cout << std::setw(7) << t << " | "
                      << "(" << std::setw(7) << true_position.x << ", "
                      << std::setw(7) << true_position.y << ", "
                      << std::setw(7) << true_position.z << ") | "
                      << "(" << std::setw(7) << est_position.x << ", "
                      << std::setw(7) << est_position.y << ", "
                      << std::setw(7) << est_position.z << ") | "
                      << std::setw(7) << pos_error << " | "
                      << std::setw(6) << uncertainty << "\n";
        }

        // Integrate true state forward
        RK4Integrator rk4;
        State current(true_position, true_velocity);
        auto accel_func = [](const State& s, double /*t*/) -> Vec3 {
            Vec3 drag = s.velocity * (-0.001 * s.velocity.magnitude());
            return Vec3(0.0, -9.81, 0.0) + drag;
        };
        State next = rk4.step(current, t, dt, accel_func);
        true_state.set_position(next.position);
        true_state.set_velocity(next.velocity);

        // Stop if projectile hits ground
        if (true_position.y < 0) {
            std::cout << "\nProjectile impacted at t = " << t << " s\n";
            break;
        }
    }

    std::cout << "\n=== Demo Complete ===\n";
    std::cout << "\nThe Extended Kalman Filter successfully tracks the projectile\n";
    std::cout << "despite noisy radar measurements. The filter combines:\n";
    std::cout << "  1. Physics-based prediction (gravity + drag)\n";
    std::cout << "  2. Radar measurement updates (noisy but informative)\n";
    std::cout << "  3. Optimal weighting via Kalman gain\n";

    return 0;
}
