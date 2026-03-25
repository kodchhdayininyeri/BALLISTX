#include "sensors/imu_model.h"
#include "ballistics/state_6dof.h"
#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>
#include <vector>
#include <fstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

void test_imu_grades() {
    std::cout << "=== IMU Grade Specifications ===\n\n";

    std::vector<IMUGrade> grades = {
        IMUGrade::CONSUMER,
        IMUGrade::INDUSTRIAL,
        IMUGrade::TACTICAL,
        IMUGrade::STRATEGIC
    };

    std::cout << std::left << std::setw(20) << "Grade"
              << std::setw(12) << "ARW (°/√hr)"
              << std::setw(15) << "Bias Inst (°/hr)"
              << std::setw(15) << "RRW (°/hr/√Hz)"
              << std::setw(15) << "Accel σ (mg/√Hz)\n";
    std::cout << std::string(80, '-') << "\n";

    for (auto grade : grades) {
        IMUNoiseModel noise = IMUNoiseModel::from_grade(grade);

        std::string grade_name;
        switch (grade) {
            case IMUGrade::CONSUMER: grade_name = "Consumer"; break;
            case IMUGrade::INDUSTRIAL: grade_name = "Industrial"; break;
            case IMUGrade::TACTICAL: grade_name = "Tactical"; break;
            case IMUGrade::STRATEGIC: grade_name = "Strategic"; break;
        }

        std::cout << std::left << std::setw(20) << grade_name
                  << std::setw(12) << noise.gyro_noise_density
                  << std::setw(15) << noise.gyro_bias_instability
                  << std::setw(15) << noise.gyro_bias_random_walk
                  << std::setw(15) << (noise.accel_noise_density * 9.80665 * 1e6) << "\n";
    }
}

void test_measurement_generation() {
    std::cout << "\n=== IMU Measurement Generation ===\n\n";

    // Create tactical grade IMU
    IMUModel imu(IMUGrade::TACTICAL, 100.0);
    std::cout << imu.get_specifications() << "\n\n";

    // Create a rotating, accelerating body (missile in flight)
    Vec3 pos(0.0, 1000.0, 0.0);
    Vec3 vel(200.0, 50.0, 0.0);
    Quaternion q = Quaternion::from_axis_angle(Vec3(0.0, 1.0, 0.0), 30.0 * M_PI / 180.0);  // 30° pitch up
    Vec3 ang_vel(0.0, 0.0, 50.0);  // 50 rad/s roll

    State6DOF state(pos, vel, q, ang_vel);

    // Get perfect measurement
    auto perfect = imu.measure_perfect(state);
    std::cout << "Perfect Measurement:\n";
    std::cout << "  Accel: " << perfect.accel << " g\n";
    std::cout << "  Gyro:  " << perfect.gyro << " °/s\n\n";

    // Generate noisy measurements
    std::cout << "Noisy Measurements (10 samples @ 100 Hz):\n";
    std::cout << "  #   Accel X (g)  Accel Y (g)  Accel Z (g)  Gyro X (°/s)  Gyro Y (°/s)  Gyro Z (°/s)\n";
    std::cout << "  " << std::string(80, '-') << "\n";

    for (int i = 0; i < 10; ++i) {
        auto z = imu.measure(state);
        std::cout << "  " << std::setw(2) << (i + 1) << " "
                  << std::setw(12) << std::fixed << std::setprecision(6) << z.accel.x << " "
                  << std::setw(12) << z.accel.y << " "
                  << std::setw(12) << z.accel.z << " "
                  << std::setw(13) << std::setprecision(4) << z.gyro.x << " "
                  << std::setw(13) << z.gyro.y << " "
                  << std::setw(13) << z.gyro.z << "\n";
    }
}

void test_noise_statistics() {
    std::cout << "\n=== Noise Statistics Verification ===\n\n";

    // Stationary IMU (should measure -g in Z axis)
    Vec3 pos(0.0, 0.0, 0.0);
    Vec3 vel(0.0, 0.0, 0.0);
    Quaternion q = Quaternion::identity();
    Vec3 ang_vel(0.0, 0.0, 0.0);
    State6DOF state(pos, vel, q, ang_vel);

    const int n_samples = 10000;

    for (auto grade : {IMUGrade::CONSUMER, IMUGrade::INDUSTRIAL, IMUGrade::TACTICAL, IMUGrade::STRATEGIC}) {
        IMUModel imu(grade, 100.0);
        auto noise_model = imu.get_noise_model();

        std::vector<double> accel_x, accel_y, accel_z;
        std::vector<double> gyro_x, gyro_y, gyro_z;

        for (int i = 0; i < n_samples; ++i) {
            auto z = imu.measure(state);
            accel_x.push_back(z.accel.x);
            accel_y.push_back(z.accel.y);
            accel_z.push_back(z.accel.z);
            gyro_x.push_back(z.gyro.x);
            gyro_y.push_back(z.gyro.y);
            gyro_z.push_back(z.gyro.z);
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

        auto [ax_mean, ax_std] = compute_stats(accel_x);
        auto [ay_mean, ay_std] = compute_stats(accel_y);
        auto [az_mean, az_std] = compute_stats(accel_z);
        auto [gx_mean, gx_std] = compute_stats(gyro_x);
        auto [gy_mean, gy_std] = compute_stats(gyro_y);
        auto [gz_mean, gz_std] = compute_stats(gyro_z);

        std::string grade_name;
        switch (grade) {
            case IMUGrade::CONSUMER: grade_name = "Consumer"; break;
            case IMUGrade::INDUSTRIAL: grade_name = "Industrial"; break;
            case IMUGrade::TACTICAL: grade_name = "Tactical"; break;
            case IMUGrade::STRATEGIC: grade_name = "Strategic"; break;
        }

        std::cout << grade_name << " Grade IMU (Stationary, 100 Hz):\n";
        std::cout << "  Accel X: mean=" << std::scientific << std::setprecision(2) << ax_mean
                  << " g, std=" << std::fixed << std::setprecision(6) << (ax_std * 1000) << " mg\n";
        std::cout << "  Accel Y: mean=" << std::scientific << ay_mean
                  << " g, std=" << std::fixed << (ay_std * 1000) << " mg\n";
        std::cout << "  Accel Z: mean=" << std::scientific << az_mean
                  << " g, std=" << std::fixed << (az_std * 1000) << " mg (expected: ~1 g for gravity)\n";
        std::cout << "  Gyro X:  mean=" << std::scientific << std::setprecision(2) << gx_mean
                  << " °/s, std=" << std::fixed << std::setprecision(4) << gx_std << " °/s\n";
        std::cout << "  Gyro Y:  mean=" << std::scientific << gy_mean
                  << " °/s, std=" << std::fixed << gy_std << " °/s\n";
        std::cout << "  Gyro Z:  mean=" << std::scientific << gz_mean
                  << " °/s, std=" << std::fixed << gz_std << " °/s\n\n";
    }
}

void test_bias_drift() {
    std::cout << "=== Bias Drift Over Time ===\n\n";

    IMUModel imu(IMUGrade::INDUSTRIAL, 100.0);

    // Stationary IMU
    Vec3 pos(0.0, 0.0, 0.0);
    Vec3 vel(0.0, 0.0, 0.0);
    Quaternion q = Quaternion::identity();
    Vec3 ang_vel(0.0, 0.0, 0.0);
    State6DOF state(pos, vel, q, ang_vel);

    std::cout << "Time (s) | Accel Bias X (mg) | Accel Bias Y (mg) | Accel Bias Z (mg) | Gyro Bias X (°/hr) | Gyro Bias Y (°/hr) | Gyro Bias Z (°/hr)\n";
    std::cout << "---------+------------------+------------------+------------------+-------------------+-------------------+-------------------\n";

    const int n_seconds = 600;  // 10 minutes
    const int print_interval = 60;

    for (int t = 0; t <= n_seconds; t += 1) {
        auto z = imu.measure(state);

        if (t % print_interval == 0) {
            auto accel_bias = imu.get_accel_bias();
            auto gyro_bias = imu.get_gyro_bias();

            std::cout << std::setw(8) << t << " | "
                      << std::setw(16) << std::fixed << std::setprecision(3) << (accel_bias.x * 1000) << " | "
                      << std::setw(16) << (accel_bias.y * 1000) << " | "
                      << std::setw(16) << (accel_bias.z * 1000) << " | "
                      << std::setw(17) << (gyro_bias.x * 3600) << " | "
                      << std::setw(17) << (gyro_bias.y * 3600) << " | "
                      << std::setw(17) << (gyro_bias.z * 3600) << "\n";
        }
    }
}

void test_allan_variance() {
    std::cout << "\n=== Allan Variance (Simplified) ===\n\n";
    std::cout << "Note: Full Allan variance computation requires specialized analysis.\n";
    std::cout << "This is a simplified demonstration showing noise characteristics.\n\n";

    IMUModel imu(IMUGrade::TACTICAL, 100.0);

    // Stationary IMU
    Vec3 pos(0.0, 0.0, 0.0);
    Vec3 vel(0.0, 0.0, 0.0);
    Quaternion q = Quaternion::identity();
    Vec3 ang_vel(0.0, 0.0, 0.0);
    State6DOF state(pos, vel, q, ang_vel);

    // Collect long data sequence
    const int n_samples = 10000;  // 100 seconds @ 100 Hz
    std::vector<double> gyro_z;

    for (int i = 0; i < n_samples; ++i) {
        auto z = imu.measure(state);
        gyro_z.push_back(z.gyro.z);
    }

    // Compute Allan variance at different cluster sizes
    std::cout << "Cluster Size (samples) | Allan Deviation (°/hr)\n";
    std::cout << "---------------------+-------------------------\n";

    std::vector<int> cluster_sizes = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000};

    for (int m : cluster_sizes) {
        if (m > n_samples / 10) break;

        // Compute Allan variance
        double sum = 0.0;
        int n_clusters = n_samples / m;

        for (int i = 0; i < n_clusters - 1; ++i) {
            // Compute cluster average
            double cluster1 = 0.0, cluster2 = 0.0;
            for (int j = 0; j < m; ++j) {
                cluster1 += gyro_z[i * m + j];
                cluster2 += gyro_z[(i + 1) * m + j];
            }
            cluster1 /= m;
            cluster2 /= m;

            double diff = cluster2 - cluster1;
            sum += diff * diff;
        }

        double allan_var = sum / (2.0 * (n_clusters - 1));
        double allan_dev = std::sqrt(allan_var);

        // Convert to °/hr
        double allan_dev_hr = allan_dev * 3600.0;

        std::cout << std::setw(20) << m << " | "
                  << std::setw(22) << std::scientific << std::setprecision(2) << allan_dev_hr << "\n";
    }
}

void test_temperature_effects() {
    std::cout << "\n=== Temperature Effects on Bias ===\n\n";

    IMUModel imu(IMUGrade::INDUSTRIAL, 100.0);

    // Stationary IMU
    Vec3 pos(0.0, 0.0, 0.0);
    Vec3 vel(0.0, 0.0, 0.0);
    Quaternion q = Quaternion::identity();
    Vec3 ang_vel(0.0, 0.0, 0.0);
    State6DOF state(pos, vel, q, ang_vel);

    std::cout << "Temperature (°C) | Accel Bias Z (mg) | Gyro Bias Z (°/hr)\n";
    std::cout << "----------------+------------------+-------------------\n";

    for (int temp = -20; temp <= 60; temp += 10) {
        imu.set_temperature(temp);
        imu.reset_bias();  // Reset bias at new temperature

        // Take measurement to update bias
        auto z = imu.measure(state);

        auto accel_bias = imu.get_accel_bias();
        auto gyro_bias = imu.get_gyro_bias();

        std::cout << std::setw(15) << temp << " | "
                  << std::setw(15) << std::fixed << std::setprecision(3) << (accel_bias.z * 1000) << " | "
                  << std::setw(18) << (gyro_bias.z * 3600) << "\n";
    }
}

void test_missile_simulation() {
    std::cout << "\n=== Missile Flight IMU Simulation ===\n\n";

    IMUModel imu(IMUGrade::TACTICAL, 200.0);  // 200 Hz update rate
    std::cout << imu.get_specifications() << "\n\n";

    // Missile initial state
    Vec3 pos(0.0, 0.0, 0.0);
    Vec3 vel(250.0, 100.0, 0.0);  // 250 m/s forward, 100 m/s up
    Quaternion q = Quaternion::identity();
    Vec3 ang_vel(0.0, 0.0, 100.0);  // 100 rad/s roll
    State6DOF state(pos, vel, q, ang_vel);

    std::cout << "Time (s) | Accel X (g) | Accel Y (g) | Accel Z (g) | Gyro Z (°/s)\n";
    std::cout << "---------+-------------+-------------+-------------+--------------\n";

    const double dt = 0.005;  // 200 Hz
    const int duration = 5;   // 5 seconds

    for (int step = 0; step <= duration / dt; ++step) {
        double t = step * dt;

        // Generate IMU measurement
        auto z = imu.measure(state);

        // Print every 0.5 seconds
        if (step % 100 == 0) {
            std::cout << std::setw(8) << std::fixed << std::setprecision(2) << t << " | "
                      << std::setw(11) << std::setprecision(4) << z.accel.x << " | "
                      << std::setw(11) << z.accel.y << " | "
                      << std::setw(11) << z.accel.z << " | "
                      << std::setw(12) << z.gyro.z << "\n";
        }

        // Update true state (simple ballistic trajectory with gravity)
        Vec3 accel(0.0, -9.80665, 0.0);  // Gravity
        vel = vel + accel * dt;
        pos = pos + vel * dt;

        state.set_position(pos);
        state.set_velocity(vel);

        // Update orientation (roll continues)
        ang_vel = Vec3(0.0, 0.0, 100.0);
        state.set_angular_velocity(ang_vel);
    }

    std::cout << "\n=== Simulation Complete ===\n";
}

void test_export_to_csv() {
    std::cout << "\n=== Export IMU Data to CSV ===\n\n";

    IMUModel imu(IMUGrade::TACTICAL, 100.0);

    // Simulate missile trajectory
    Vec3 pos(0.0, 0.0, 0.0);
    Vec3 vel(250.0, 150.0, 0.0);
    Quaternion q = Quaternion::identity();
    Vec3 ang_vel(0.0, 0.0, 100.0);
    State6DOF state(pos, vel, q, ang_vel);

    std::ofstream file("imu_data.csv");
    file << "Time(s),AccelX(g),AccelY(g),AccelZ(g),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s)\n";

    const double dt = 0.01;  // 100 Hz
    const int duration = 10;

    for (int step = 0; step <= duration / dt; ++step) {
        double t = step * dt;
        auto z = imu.measure(state);

        file << t << ","
             << z.accel.x << "," << z.accel.y << "," << z.accel.z << ","
             << z.gyro.x << "," << z.gyro.y << "," << z.gyro.z << "\n";

        // Update state
        Vec3 accel(0.0, -9.80665, 0.0);
        vel = vel + accel * dt;
        pos = pos + vel * dt;
        state.set_position(pos);
        state.set_velocity(vel);
    }

    file.close();
    std::cout << "IMU data exported to: imu_data.csv\n";
    std::cout << "You can plot this data to visualize noise characteristics.\n";
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                IMU Model Test Suite                            ║\n";
    std::cout << "║          Inertial Measurement Unit Simulation                 ║\n";
    std::cout << "║     Accelerometer + Gyroscope with Realistic Errors           ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

    test_imu_grades();
    test_measurement_generation();
    test_noise_statistics();
    test_bias_drift();
    test_allan_variance();
    test_temperature_effects();
    test_missile_simulation();
    test_export_to_csv();

    std::cout << "\n=== All IMU Tests Complete ===\n";
    return 0;
}
