#include "sensors/radar_model.h"
#include "sensors/kalman_filter.h"
#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper to convert SimpleRadarMeasurement to RadarMeasurement (for EKF)
ballistx::RadarMeasurement to_ekf_measurement(const ballistx::SimpleRadarMeasurement& m) {
    ballistx::RadarMeasurement ekf_m;
    ekf_m.range = m.range;
    ekf_m.azimuth = m.azimuth;
    ekf_m.elevation = m.elevation;
    ekf_m.range_rate = m.range_rate;
    return ekf_m;
}

ballistx::RadarMeasurement to_ekf_measurement(const ballistx::RadarModel::Measurement& m) {
    ballistx::RadarMeasurement ekf_m;
    ekf_m.range = m.range;
    ekf_m.azimuth = m.azimuth;
    ekf_m.elevation = m.elevation;
    ekf_m.range_rate = m.range_rate;
    return ekf_m;
}

void test_radar_types() {
    std::cout << "=== Radar Type Specifications ===\n\n";

    std::vector<ballistx::RadarType> types = {
        ballistx::RadarType::GENERIC,
        ballistx::RadarType::ATC_SURVEILLANCE,
        ballistx::RadarType::FIRE_CONTROL,
        ballistx::RadarType::MISSILE_GUIDANCE,
        ballistx::RadarType::PHASED_ARRAY,
        ballistx::RadarType::PRECISION_APPROACH
    };

    std::cout << std::left << std::setw(25) << "Radar Type"
              << std::setw(12) << "Range (m)"
              << std::setw(12) << "Az (deg)"
              << std::setw(12) << "El (deg)"
              << std::setw(12) << "RR (m/s)\n";
    std::cout << std::string(73, '-') << "\n";

    for (auto type : types) {
        ballistx::RadarModel radar(type);
        auto noise = radar.get_noise_model();

        std::string type_name;
        switch (type) {
            case ballistx::RadarType::GENERIC: type_name = "Generic"; break;
            case ballistx::RadarType::ATC_SURVEILLANCE: type_name = "ATC Surveillance"; break;
            case ballistx::RadarType::FIRE_CONTROL: type_name = "Fire Control"; break;
            case ballistx::RadarType::MISSILE_GUIDANCE: type_name = "Missile Guidance"; break;
            case ballistx::RadarType::PHASED_ARRAY: type_name = "Phased Array"; break;
            case ballistx::RadarType::PRECISION_APPROACH: type_name = "Precision Approach"; break;
        }

        std::cout << std::left << std::setw(25) << type_name
                  << std::setw(12) << noise.range_std
                  << std::setw(12) << (noise.azimuth_std * 180.0 / M_PI)
                  << std::setw(12) << (noise.elevation_std * 180.0 / M_PI)
                  << std::setw(12) << noise.range_rate_std << "\n";
    }
}

void test_measurement_generation() {
    std::cout << "\n=== Measurement Generation Test ===\n\n";

    // Create fire control radar
    ballistx::RadarModel radar(ballistx::RadarType::FIRE_CONTROL);
    std::cout << radar.get_specifications() << "\n\n";

    // Target at 10km range, 3km altitude
    ballistx::Vec3 target_pos(8000.0, 3000.0, 6000.0);
    ballistx::Vec3 target_vel(250.0, 0.0, 100.0);
    ballistx::State6DOF target(target_pos, target_vel, ballistx::Quaternion::identity(), ballistx::Vec3::zero());

    std::cout << "Target State:\n";
    std::cout << "  Position: " << target_pos << " m\n";
    std::cout << "  Velocity: " << target_vel << " m/s\n";
    std::cout << "  Range: " << target_pos.magnitude() << " m\n\n";

    // Generate perfect measurement
    auto perfect = radar.measure_perfect(target);
    std::cout << "Perfect Measurement:\n";
    std::cout << "  Range: " << std::fixed << std::setprecision(2) << perfect.range << " m\n";
    std::cout << "  Azimuth: " << (perfect.azimuth * 180.0 / M_PI) << "°\n";
    std::cout << "  Elevation: " << (perfect.elevation * 180.0 / M_PI) << "°\n";
    std::cout << "  Range Rate: " << perfect.range_rate << " m/s\n\n";

    // Generate noisy measurements
    std::cout << "Noisy Measurements (10 samples):\n";
    std::cout << "  #   Range (m)  Az (°)   El (°)   RR (m/s)\n";
    std::cout << "  " << std::string(42, '-') << "\n";

    for (int i = 0; i < 10; ++i) {
        auto z = radar.measure(target);
        std::cout << "  " << std::setw(2) << (i + 1) << " "
                  << std::setw(10) << std::fixed << std::setprecision(2) << z.range << " "
                  << std::setw(7) << std::setprecision(3) << (z.azimuth * 180.0 / M_PI) << " "
                  << std::setw(7) << (z.elevation * 180.0 / M_PI) << " "
                  << std::setw(8) << z.range_rate << "\n";
    }
}

void test_noise_statistics() {
    std::cout << "\n=== Noise Statistics Verification ===\n\n";

    ballistx::Vec3 target_pos(10000.0, 5000.0, 8000.0);
    ballistx::Vec3 target_vel(200.0, 50.0, 100.0);
    ballistx::State6DOF target(target_pos, target_vel, ballistx::Quaternion::identity(), ballistx::Vec3::zero());

    const int n_samples = 10000;

    for (auto radar_type : {ballistx::RadarType::ATC_SURVEILLANCE, ballistx::RadarType::FIRE_CONTROL, ballistx::RadarType::MISSILE_GUIDANCE}) {
        ballistx::RadarModel radar(radar_type);
        auto noise_model = radar.get_noise_model();

        std::vector<double> ranges, azimuths, elevations, range_rates;
        int detections = 0;

        for (int i = 0; i < n_samples; ++i) {
            auto z = radar.measure(target);
            if (z.detected) {
                detections++;
                ranges.push_back(z.range);
                azimuths.push_back(z.azimuth);
                elevations.push_back(z.elevation);
                range_rates.push_back(z.range_rate);
            }
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

        auto [r_mean, r_std] = compute_stats(ranges);
        auto [az_mean, az_std] = compute_stats(azimuths);
        auto [el_mean, el_std] = compute_stats(elevations);
        auto [rr_mean, rr_std] = compute_stats(range_rates);

        // Get perfect measurement for comparison
        auto perfect = radar.measure_perfect(target);

        std::cout << radar.get_specifications() << "\n";
        std::cout << "  Detection Rate: " << (100.0 * detections / n_samples) << "%\n";
        std::cout << "  Range:       mean=" << std::fixed << std::setprecision(2)
                  << std::setw(10) << r_mean << " m (true: " << perfect.range << " m), "
                  << "std=" << std::setw(6) << r_std << " m (expected: " << noise_model.range_std << " m)\n";
        std::cout << "  Azimuth:     mean=" << std::setprecision(4)
                  << std::setw(10) << (az_mean * 180.0 / M_PI) << "° (true: " << (perfect.azimuth * 180.0 / M_PI) << "°), "
                  << "std=" << std::setw(6) << (az_std * 180.0 / M_PI) << "° (expected: " << (noise_model.azimuth_std * 180.0 / M_PI) << "°)\n";
        std::cout << "  Elevation:   mean=" << std::setw(10) << (el_mean * 180.0 / M_PI) << "° (true: " << (perfect.elevation * 180.0 / M_PI) << "°), "
                  << "std=" << std::setw(6) << (el_std * 180.0 / M_PI) << "° (expected: " << (noise_model.elevation_std * 180.0 / M_PI) << "°)\n";
        std::cout << "  Range Rate:  mean=" << std::setprecision(2)
                  << std::setw(10) << rr_mean << " m/s (true: " << perfect.range_rate << " m/s), "
                  << "std=" << std::setw(6) << rr_std << " m/s (expected: " << noise_model.range_rate_std << " m/s)\n\n";
    }
}

void test_detection_probability() {
    std::cout << "=== Detection Probability vs Range ===\n\n";

    ballistx::RadarModel radar(ballistx::RadarType::FIRE_CONTROL);
    radar.set_detection_probability(0.9);
    radar.set_max_range(50000.0);  // 50 km

    std::cout << "Range (km) | Detection Rate\n";
    std::cout << "-----------+----------------\n";

    const int trials = 1000;
    for (double range_km = 5.0; range_km <= 50.0; range_km += 5.0) {
        ballistx::Vec3 pos(range_km * 1000.0, 0.0, 0.0);
        ballistx::State6DOF target(pos, ballistx::Vec3(200.0, 0.0, 0.0), ballistx::Quaternion::identity(), ballistx::Vec3::zero());

        int detections = 0;
        for (int i = 0; i < trials; ++i) {
            if (radar.measure(target).detected) {
                detections++;
            }
        }

        std::cout << std::setw(10) << std::fixed << std::setprecision(1) << range_km << " | "
                  << std::setw(14) << std::setprecision(3) << (100.0 * detections / trials) << "%\n";
    }
}

void test_clutter_simulation() {
    std::cout << "\n=== Clutter (False Target) Simulation ===\n\n";

    ballistx::RadarModel radar(ballistx::RadarType::ATC_SURVEILLANCE);

    ballistx::Vec3 target_pos(15000.0, 3000.0, 10000.0);
    ballistx::Vec3 target_vel(250.0, 20.0, 50.0);
    ballistx::State6DOF target(target_pos, target_vel, ballistx::Quaternion::identity(), ballistx::Vec3::zero());

    std::cout << "Simulating radar scan with clutter...\n";
    std::cout << "True target range: " << target_pos.magnitude() << " m\n\n";

    // Generate measurements with clutter
    auto measurements = radar.measure_with_clutter(target, 5);

    std::cout << "Detected returns (including clutter):\n";
    std::cout << "  #   Range (m)  Azimuth (°)  Elevation (°)  Type\n";
    std::cout << "  " << std::string(50, '-') << "\n";

    for (size_t i = 0; i < measurements.size(); ++i) {
        const auto& m = measurements[i];
        std::cout << "  " << std::setw(2) << (i + 1) << " "
                  << std::setw(10) << std::fixed << std::setprecision(1) << m.range << " "
                  << std::setw(11) << std::setprecision(3) << (m.azimuth * 180.0 / M_PI) << " "
                  << std::setw(13) << (m.elevation * 180.0 / M_PI);

        // Check if this is close to true target (within 5% range)
        auto true_meas = radar.measure_perfect(target);
        if (std::abs(m.range - true_meas.range) < 0.05 * true_meas.range) {
            std::cout << "  TRUE TARGET";
        } else {
            std::cout << "  CLUTTER";
        }
        std::cout << "\n";
    }
}

void test_ekf_with_radar() {
    std::cout << "\n=== EKF Tracking with Radar Measurements ===\n\n";

    // Create fire control radar
    ballistx::RadarModel radar(ballistx::RadarType::FIRE_CONTROL);
    ballistx::Vec3 radar_pos(0.0, 0.0, 0.0);
    radar.set_position(radar_pos);

    // Target: aircraft at 10km
    ballistx::Vec3 true_pos(8000.0, 2000.0, 6000.0);
    ballistx::Vec3 true_vel(200.0, 0.0, 50.0);
    ballistx::State6DOF true_state(true_pos, true_vel, ballistx::Quaternion::identity(), ballistx::Vec3::zero());

    // Initialize EKF with rough estimate
    ballistx::State6DOF initial_estimate(
        ballistx::Vec3(7500.0, 1800.0, 5500.0),  // 500m position error
        ballistx::Vec3(180.0, 10.0, 40.0),       // Velocity error
        ballistx::Quaternion::identity(),
        ballistx::Vec3::zero()
    );

    ballistx::ExtendedKalmanFilter ekf(initial_estimate, 100.0);  // Daha küçük initial covariance
    ekf.set_process_noise(0.01, 0.05);
    ekf.set_measurement_noise(to_ekf_measurement(radar.get_noise_model().as_measurement()));

    const double dt = 0.1;
    const int n_steps = 100;

    std::cout << "Time(s) | True X (m) | True Y (m) | Est X (m) | Est Y (m) | Pos Err (m)\n";
    std::cout << "--------+------------+------------+------------+------------+-------------\n";

    for (int step = 0; step <= n_steps; ++step) {
        double t = step * dt;

        // Get true state BEFORE update
        ballistx::Vec3 pos = true_state.get_position();
        ballistx::Vec3 vel = true_state.get_velocity();

        // STEP 1: PREDICT - propagate state forward
        ballistx::Vec3 accel(0.0, 0.0, 0.0);  // Constant velocity model
        ekf.predict(dt, [accel](ballistx::Vec3, ballistx::Vec3, double) -> ballistx::Vec3 { return accel; });

        // STEP 2: MEASURE - get radar measurement from current true state
        auto z = radar.measure(true_state);
        if (z.detected) {
            // STEP 3: UPDATE - correct prediction with measurement
            ekf.update(to_ekf_measurement(z));
        }

        // Get estimate and compute error
        ballistx::Vec3 est_pos = ekf.get_position();
        double pos_error = pos.distance_to(est_pos);

        // Print every 10 steps
        if (step % 10 == 0) {
            std::cout << std::setw(7) << std::fixed << std::setprecision(1) << t << " | "
                      << std::setw(10) << pos.x << " | "
                      << std::setw(10) << pos.y << " | "
                      << std::setw(10) << est_pos.x << " | "
                      << std::setw(10) << est_pos.y << " | "
                      << std::setw(11) << pos_error << "\n";
        }

        // Update true state for next iteration
        true_state.set_position(pos + vel * dt);
    }

    std::cout << "\n=== Tracking Complete ===\n";
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║              Radar Model Test Suite                            ║\n";
    std::cout << "║          Real-World Radar Specifications                      ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

    test_radar_types();
    test_measurement_generation();
    test_noise_statistics();
    test_detection_probability();
    test_clutter_simulation();
    test_ekf_with_radar();

    std::cout << "\n=== All Radar Tests Complete ===\n";
    return 0;
}
