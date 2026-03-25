/**
 * @file kalman_visualization_demo.cpp
 * @brief Extended Kalman Filter demo with CSV export for visualization
 *
 * This demo generates tracking data and exports it to CSV files for
 * visualization in Python using Plotly. It demonstrates:
 * - True projectile trajectory
 * - Noisy radar measurements
 * - Kalman filter estimates
 * - Position uncertainty over time
 */

#include "sensors/kalman_filter.h"
#include "utils/integrator.h"
#include "utils/vec3.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <random>
#include <cmath>
#include <vector>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

// Process model that matches the true dynamics (gravity + drag)
// Returns acceleration given position, velocity, and time
Vec3 projectile_accel(Vec3 position, Vec3 velocity, double /*t*/) {
    // Gravity
    Vec3 gravity(0.0, -9.81, 0.0);

    // Simple drag model
    double speed = velocity.magnitude();
    double drag_factor = 0.001;  // Simplified drag coefficient
    Vec3 drag = velocity * (-drag_factor * speed);

    return gravity + drag;
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     Kalman Filter Visualization Demo - Data Export            ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

    // Simulation parameters
    const double dt = 0.01;
    const double duration = 10.0;
    const size_t num_steps = static_cast<size_t>(duration / dt);

    // True initial state (artillery shell)
    Vec3 true_pos(0.0, 0.0, 0.0);
    Vec3 true_vel(250.0, 150.0, 50.0);
    Quaternion true_quat = Quaternion::identity();
    Vec3 true_ang_vel(0.0, 0.0, 0.0);

    State6DOF true_state(true_pos, true_vel, true_quat, true_ang_vel);

    // Radar noise parameters (typical values for a military radar)
    const double range_std = 15.0;        // 15m range error
    const double azimuth_std = 0.003;     // ~0.17° azimuth error
    const double elevation_std = 0.002;   // ~0.11° elevation error
    const double range_rate_std = 2.0;    // 2 m/s range rate error

    std::random_device rd;
    std::mt19937 gen(42);  // Fixed seed for reproducibility
    std::normal_distribution<> noise(0.0, 1.0);

    // Initialize EKF with initial estimate (slightly different from true state)
    State6DOF initial_estimate(
        Vec3(3.0, 3.0, 1.0),          // ~4.4m position error (reduced from 35m)
        Vec3(240.0, 145.0, 48.0),     // ~12 m/s velocity error (reduced from 20 m/s)
        Quaternion::identity(),
        Vec3(0.0, 0.0, 0.0)
    );

    ExtendedKalmanFilter ekf(initial_estimate, 50.0);  // 50m initial uncertainty

    // Configure process noise (model uncertainty)
    // Lower values make the filter more confident in the process model
    // This allows uncertainty to decrease over time as measurements confirm the model
    ekf.set_process_noise(0.01, 0.01);  // Reduced from 0.5, 0.2

    // Configure measurement noise (radar specifications)
    RadarMeasurement R;
    R.range = range_std;
    R.azimuth = azimuth_std;
    R.elevation = elevation_std;
    R.range_rate = range_rate_std;
    ekf.set_measurement_noise(R);

    std::cout << "Simulation Parameters:\n";
    std::cout << "  Duration: " << duration << " seconds\n";
    std::cout << "  Time step: " << dt << " seconds\n";
    std::cout << "  Total steps: " << num_steps << "\n\n";

    std::cout << "Initial State:\n";
    std::cout << "  True Position:  " << true_state.get_position() << " m\n";
    std::cout << "  True Velocity:  " << true_state.get_velocity() << " m/s\n";
    std::cout << "  Est Position:   " << ekf.get_position() << " m\n";
    std::cout << "  Est Velocity:   " << ekf.get_velocity() << " m/s\n";
    std::cout << "  Initial Uncertainty: " << ekf.get_position_uncertainty() << " m\n\n";

    std::cout << "Radar Noise Parameters:\n";
    std::cout << "  Range STD:      " << range_std << " m\n";
    std::cout << "  Azimuth STD:    " << (azimuth_std * 180.0 / M_PI) << " deg\n";
    std::cout << "  Elevation STD:  " << (elevation_std * 180.0 / M_PI) << " deg\n";
    std::cout << "  Range Rate STD: " << range_rate_std << " m/s\n\n";

    // Open CSV files
    std::ofstream trajectory_file("kalman_trajectory.csv");
    std::ofstream measurements_file("kalman_measurements.csv");
    std::ofstream errors_file("kalman_errors.csv");

    // Write headers
    trajectory_file << "time,true_x,true_y,true_z,est_x,est_y,est_z,uncertainty\n";
    measurements_file << "time,range,azimuth,elevation,range_rate\n";
    errors_file << "time,pos_error,vel_error,true_v,est_v\n";

    // Data storage for analysis
    std::vector<double> times;
    std::vector<double> pos_errors;
    std::vector<double> vel_errors;
    std::vector<double> uncertainties;
    std::vector<double> true_speeds;
    std::vector<double> est_speeds;

    std::cout << "Running simulation...\n";

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
        ekf.predict(dt, projectile_accel);

        // EKF update step (incorporate measurement)
        ekf.update(z);

        // Get filtered estimate
        Vec3 est_position = ekf.get_position();
        Vec3 est_velocity = ekf.get_velocity();
        double uncertainty = ekf.get_position_uncertainty();

        // Compute errors
        double pos_error = true_position.distance_to(est_position);
        double vel_error = true_velocity.distance_to(est_velocity);

        // Store data
        times.push_back(t);
        pos_errors.push_back(pos_error);
        vel_errors.push_back(vel_error);
        uncertainties.push_back(uncertainty);
        true_speeds.push_back(true_velocity.magnitude());
        est_speeds.push_back(est_velocity.magnitude());

        // Write to trajectory file
        trajectory_file << t << ","
                       << true_position.x << "," << true_position.y << "," << true_position.z << ","
                       << est_position.x << "," << est_position.y << "," << est_position.z << ","
                       << uncertainty << "\n";

        // Write to measurements file
        measurements_file << t << ","
                         << z.range << ","
                         << z.azimuth << ","
                         << z.elevation << ","
                         << z.range_rate << "\n";

        // Write to errors file
        errors_file << t << ","
                   << pos_error << ","
                   << vel_error << ","
                   << true_velocity.magnitude() << ","
                   << est_velocity.magnitude() << "\n";

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
            std::cout << "  Projectile impacted at t = " << t << " s\n";
            break;
        }
    }

    // Close files
    trajectory_file.close();
    measurements_file.close();
    errors_file.close();

    // Compute statistics
    auto compute_stats = [](const std::vector<double>& v) {
        double mean = 0.0, max_val = 0.0;
        for (double x : v) {
            mean += x;
            if (x > max_val) max_val = x;
        }
        mean /= v.size();
        double var = 0.0;
        for (double x : v) var += (x - mean) * (x - mean);
        var /= v.size();
        return std::make_tuple(mean, std::sqrt(var), max_val);
    };

    auto [pos_mean, pos_std, pos_max] = compute_stats(pos_errors);
    auto [vel_mean, vel_std, vel_max] = compute_stats(vel_errors);
    auto [unc_mean, unc_std, unc_max] = compute_stats(uncertainties);

    std::cout << "\n=== Statistics ===\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Position Error:\n";
    std::cout << "  Mean: " << pos_mean << " m\n";
    std::cout << "  STD:  " << pos_std << " m\n";
    std::cout << "  Max:  " << pos_max << " m\n\n";

    std::cout << "Velocity Error:\n";
    std::cout << "  Mean: " << vel_mean << " m/s\n";
    std::cout << "  STD:  " << vel_std << " m/s\n";
    std::cout << "  Max:  " << vel_max << " m/s\n\n";

    std::cout << "Uncertainty (3σ):\n";
    std::cout << "  Mean: " << unc_mean << " m\n";
    std::cout << "  STD:  " << unc_std << " m\n";
    std::cout << "  Max:  " << unc_max << " m\n\n";

    std::cout << "=== Files Generated ===\n";
    std::cout << "  kalman_trajectory.csv   - True and estimated positions\n";
    std::cout << "  kalman_measurements.csv - Noisy radar measurements\n";
    std::cout << "  kalman_errors.csv      - Position and velocity errors\n\n";

    std::cout << "Run the following command to visualize:\n";
    std::cout << "  python kalman_visualize.py\n";

    return 0;
}
