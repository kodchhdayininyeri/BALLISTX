#include "sensors/kalman_filter.h"
#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

// Simple projectile motion model for EKF process
// Returns acceleration given position and velocity
Vec3 simple_projectile_accel(Vec3 position, Vec3 velocity, double /*t*/) {
    // Gravity
    Vec3 gravity(0.0, -9.81, 0.0);

    // Simple drag: F_drag = -0.5 * rho * Cd * A * |v| * v
    double speed = velocity.magnitude();
    double drag_factor = 0.001;  // Simplified drag coefficient

    Vec3 drag = velocity * (-drag_factor * speed);
    Vec3 accel = gravity + drag;

    return accel;
}

// Simulate a projectile with noisy radar measurements
void test_ekf_tracking() {
    std::cout << "=== Extended Kalman Filter - Projectile Tracking Test ===\n\n";

    // Simulation parameters
    const double dt = 0.01;           // 10ms time step
    const double duration = 10.0;     // 10 seconds
    const size_t num_steps = static_cast<size_t>(duration / dt);

    // True initial state (artillery shell)
    Vec3 true_pos(0.0, 0.0, 0.0);
    Vec3 true_vel(250.0, 150.0, 0.0);  // 250 m/s horizontal, 150 m/s up
    Quaternion true_quat = Quaternion::identity();
    Vec3 true_ang_vel(0.0, 0.0, 100.0);  // 100 rad/s spin

    State6DOF true_state(true_pos, true_vel, true_quat, true_ang_vel);

    // Radar noise parameters
    const double range_std = 10.0;        // 10m range noise
    const double azimuth_std = 0.002;     // ~0.1° azimuth noise
    const double elevation_std = 0.002;   // ~0.1° elevation noise
    const double range_rate_std = 1.0;    // 1 m/s range rate noise

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 1.0);

    // Initialize EKF with noisy initial estimate (closer to true state)
    State6DOF initial_estimate(
        Vec3(10.0, 5.0, 0.0),          // Small position error
        Vec3(245.0, 145.0, 0.0),       // Small velocity error
        Quaternion::identity(),
        Vec3(0.0, 0.0, 0.0)
    );

    ExtendedKalmanFilter ekf(initial_estimate, 50.0);  // 50m initial uncertainty

    // Set process noise (model uncertainty)
    ekf.set_process_noise(1.0, 0.5);  // Higher process noise to trust measurements more

    // Set measurement noise (radar accuracy)
    RadarMeasurement R;
    R.range = range_std;
    R.azimuth = azimuth_std;
    R.elevation = elevation_std;
    R.range_rate = range_rate_std;
    ekf.set_measurement_noise(R);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time(s) | True X (m) | True Y (m) | Est X (m) | Est Y (m) | Pos Err (m) | Vel Err (m/s) | Uncertainty (m)\n";
    std::cout << "--------+------------+------------+------------+------------+-------------+---------------+----------------\n";

    // Tracking loop
    for (size_t step = 0; step <= num_steps; ++step) {
        double t = step * dt;

        // Get true state
        Vec3 true_position = true_state.get_position();
        Vec3 true_velocity = true_state.get_velocity();

        // Generate noisy radar measurement
        RadarMeasurement z = RadarMeasurement::from_state(true_state);
        z.range += range_std * noise(gen);
        z.azimuth += azimuth_std * noise(gen);
        z.elevation += elevation_std * noise(gen);
        z.range_rate += range_rate_std * noise(gen);

        // EKF prediction step
        ekf.predict(dt, simple_projectile_accel);

        // EKF update step
        ekf.update(z);

        // Get filtered estimate
        Vec3 est_position = ekf.get_position();
        Vec3 est_velocity = ekf.get_velocity();
        double uncertainty = ekf.get_position_uncertainty();

        // Compute errors
        double pos_error = true_position.distance_to(est_position);
        double vel_error = true_velocity.distance_to(est_velocity);

        // Print every 100 steps
        if (step % 100 == 0 || step == num_steps) {
            std::cout << std::setw(7) << t << " | "
                      << std::setw(10) << true_position.x << " | "
                      << std::setw(10) << true_position.y << " | "
                      << std::setw(10) << est_position.x << " | "
                      << std::setw(10) << est_position.y << " | "
                      << std::setw(11) << pos_error << " | "
                      << std::setw(13) << vel_error << " | "
                      << std::setw(14) << uncertainty << "\n";
        }

        // Integrate true state forward using simple Euler integration
        Vec3 accel = simple_projectile_accel(true_position, true_velocity, t);
        true_position = true_position + true_velocity * dt;
        true_velocity = true_velocity + accel * dt;
        true_state.set_position(true_position);
        true_state.set_velocity(true_velocity);

        // Stop if projectile hits ground
        if (true_position.y < 0) {
            std::cout << "\nProjectile impacted at t = " << t << " s\n";
            break;
        }
    }

    std::cout << "\n=== Test Complete ===\n";
}

void test_jacobian_computation() {
    std::cout << "\n=== Jacobian Matrix Test ===\n\n";

    // Create a test state
    State6DOF state(
        Vec3(1000.0, 500.0, 2000.0),   // Position
        Vec3(100.0, 50.0, 80.0),       // Velocity
        Quaternion::identity(),        // Orientation
        Vec3(0.1, 0.05, 0.2)           // Angular velocity
    );

    // Get predicted measurement
    Vec3 pos = state.get_position();
    double r = pos.magnitude();

    std::cout << "Test State:\n";
    std::cout << "  Position: " << pos << "\n";
    std::cout << "  Range: " << r << " m\n\n";

    // Compute measurement Jacobian numerically
    const double epsilon = 1e-6;
    std::array<double, 4> h_plus{}, h_minus{};

    std::cout << "Measurement Jacobian H (numerical):\n";
    std::cout << "            range      azimuth    elevation  range_rate\n";

    for (size_t i = 0; i < 6; ++i) {  // Only position and velocity affect measurements
        State6DOF state_plus = state;
        State6DOF state_minus = state;

        auto arr = state.get_state();
        arr[i] += epsilon;
        state_plus.set_state(arr);

        arr = state.get_state();
        arr[i] -= epsilon;
        state_minus.set_state(arr);

        auto z_plus = RadarMeasurement::from_state(state_plus);
        auto z_minus = RadarMeasurement::from_state(state_minus);

        double dr_dxi = (z_plus.range - z_minus.range) / (2 * epsilon);
        double daz_dxi = (z_plus.azimuth - z_minus.azimuth) / (2 * epsilon);
        double del_dxi = (z_plus.elevation - z_minus.elevation) / (2 * epsilon);
        double drr_dxi = (z_plus.range_rate - z_minus.range_rate) / (2 * epsilon);

        std::string name;
        if (i == 0) name = "px     ";
        else if (i == 1) name = "py     ";
        else if (i == 2) name = "pz     ";
        else if (i == 3) name = "vx     ";
        else if (i == 4) name = "vy     ";
        else name = "vz     ";

        std::cout << name << " | "
                  << std::fixed << std::setprecision(6)
                  << std::setw(10) << dr_dxi << " "
                  << std::setw(10) << daz_dxi << " "
                  << std::setw(10) << del_dxi << " "
                  << std::setw(10) << drr_dxi << "\n";
    }
}

void test_innovation_statistics() {
    std::cout << "\n=== Innovation (Measurement Residual) Statistics ===\n\n";

    const int n_samples = 1000;
    const double range_std = 50.0;
    const double az_std = 0.01;
    const double el_std = 0.005;
    const double rr_std = 3.0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 1.0);

    // Create steady target
    Vec3 target_pos(5000.0, 2000.0, 3000.0);
    Vec3 target_vel(100.0, 50.0, 80.0);

    State6DOF true_state(target_pos, target_vel, Quaternion::identity(), Vec3(0.0, 0.0, 0.0));

    ExtendedKalmanFilter ekf(true_state, 10.0);
    ekf.set_process_noise(0.1, 0.01);

    RadarMeasurement R;
    R.range = range_std;
    R.azimuth = az_std;
    R.elevation = el_std;
    R.range_rate = rr_std;
    ekf.set_measurement_noise(R);

    // Collect innovations
    std::vector<double> innov_range, innov_az, innov_el, innov_rr;

    for (int i = 0; i < n_samples; ++i) {
        // Update true state (constant velocity motion)
        Vec3 pos = true_state.get_position();
        Vec3 vel = true_state.get_velocity();
        pos = pos + vel * 0.1;  // dt = 0.1
        true_state.set_position(pos);

        RadarMeasurement z = RadarMeasurement::from_state(true_state);
        z.range += range_std * noise(gen);
        z.azimuth += az_std * noise(gen);
        z.elevation += el_std * noise(gen);
        z.range_rate += rr_std * noise(gen);

        ekf.predict_kinematic(0.1);
        ekf.update(z);

        auto innov = ekf.get_innovation();
        innov_range.push_back(innov[0]);
        innov_az.push_back(innov[1]);
        innov_el.push_back(innov[2]);
        innov_rr.push_back(innov[3]);
    }

    // Compute statistics
    auto compute_stats = [](const std::vector<double>& v) {
        double mean = 0.0, var = 0.0;
        for (double x : v) mean += x;
        mean /= v.size();
        for (double x : v) var += (x - mean) * (x - mean);
        var /= v.size();
        return std::make_pair(mean, std::sqrt(var));
    };

    auto [range_mean, range_std_err] = compute_stats(innov_range);
    auto [az_mean, az_std_err] = compute_stats(innov_az);
    auto [el_mean, el_std_err] = compute_stats(innov_el);
    auto [rr_mean, rr_std_err] = compute_stats(innov_rr);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Innovation Statistics (should have mean ≈ 0, std ≈ measurement noise):\n\n";
    std::cout << "Range:      mean = " << std::setw(10) << range_mean
              << ", std = " << std::setw(10) << range_std_err << " (expected: " << range_std << ")\n";
    std::cout << "Azimuth:    mean = " << std::setw(10) << az_mean
              << ", std = " << std::setw(10) << az_std_err << " (expected: " << az_std << ")\n";
    std::cout << "Elevation:  mean = " << std::setw(10) << el_mean
              << ", std = " << std::setw(10) << el_std_err << " (expected: " << el_std << ")\n";
    std::cout << "Range Rate: mean = " << std::setw(10) << rr_mean
              << ", std = " << std::setw(10) << rr_std_err << " (expected: " << rr_std << ")\n";
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║          Extended Kalman Filter (EKF) Test Suite              ║\n";
    std::cout << "║                  6-DOF State Estimation                       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

    test_ekf_tracking();
    test_jacobian_computation();
    test_innovation_statistics();

    std::cout << "\n=== All Tests Complete ===\n";
    return 0;
}
