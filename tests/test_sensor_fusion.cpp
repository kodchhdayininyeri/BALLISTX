#include "sensors/sensor_fusion.h"
#include "sensors/imu_model.h"
#include "sensors/radar_model.h"
#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

void test_gps_only_fusion() {
    std::cout << "=== GPS-Only Navigation ===\n\n";

    // Initialize fusion with rough estimate
    NavState initial;
    initial.position = Vec3(100.0, 100.0, 100.0);  // 100m error
    initial.velocity = Vec3(190.0, 40.0, 10.0);     // Velocity error
    initial.attitude = Quaternion::identity();
    initial.timestamp = 0.0;

    SensorFusion fusion(initial, 100.0, 5.0);

    // True state (aircraft in level flight)
    Vec3 true_pos(0.0, 1000.0, 0.0);
    Vec3 true_vel(200.0, 0.0, 0.0);
    Quaternion true_att = Quaternion::identity();
    Vec3 true_ang_vel(0.0, 0.0, 0.0);
    State6DOF true_state(true_pos, true_vel, true_att, true_ang_vel);

    std::cout << "Time(s) | True X (m) | True Y (m) | Est X (m) | Est Y (m) | Pos Err (m) | GPS Available\n";
    std::cout << "--------+------------+------------+------------+------------+-------------+---------------\n";

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 1.0);

    const double dt = 0.1;  // 10 Hz
    const int n_steps = 100;

    for (int step = 0; step <= n_steps; ++step) {
        double t = step * dt;

        // Update true state (constant velocity)
        true_pos = true_pos + true_vel * dt;
        true_state.set_position(true_pos);

        // Generate GPS measurement (every 1 second)
        if (step % 10 == 0) {
            GPSMeasurement gps = GPSMeasurement::from_state(true_state);
            gps.timestamp = t;
            gps.position.x += 5.0 * noise(gen);  // 5m noise
            gps.position.y += 5.0 * noise(gen);
            gps.position.z += 5.0 * noise(gen);
            gps.velocity.x += 0.1 * noise(gen);  // 0.1 m/s noise
            gps.velocity.y += 0.1 * noise(gen);
            gps.velocity.z += 0.1 * noise(gen);
            gps.timestamp = t;

            fusion.update_with_gps(gps);
        }

        // Kinematic prediction
        fusion.predict_kinematic(dt);

        // Get estimate
        Vec3 est_pos = fusion.get_position();
        double pos_error = true_pos.distance_to(est_pos);

        // Print every 1 second
        if (step % 10 == 0) {
            std::cout << std::setw(7) << std::fixed << std::setprecision(1) << t << " | "
                      << std::setw(10) << true_pos.x << " | "
                      << std::setw(10) << true_pos.y << " | "
                      << std::setw(10) << est_pos.x << " | "
                      << std::setw(10) << est_pos.y << " | "
                      << std::setw(11) << pos_error << " | "
                      << std::setw(13) << (fusion.gps_available() ? "Yes" : "No") << "\n";
        }
    }

    std::cout << "\n";
}

void test_imu_dead_reckoning() {
    std::cout << "=== IMU Dead Reckoning (GPS Outage) ===\n\n";

    // Tactical grade IMU
    IMUModel imu(IMUGrade::TACTICAL, 100.0);
    std::cout << imu.get_specifications() << "\n\n";

    // Initialize fusion with accurate state (GPS just locked)
    NavState initial;
    initial.position = Vec3(0.0, 1000.0, 0.0);
    initial.velocity = Vec3(200.0, 0.0, 0.0);
    initial.attitude = Quaternion::identity();
    initial.timestamp = 0.0;

    SensorFusion fusion(initial, 1.0, 0.1);  // Low initial uncertainty

    // True state (aircraft in 30° climb)
    Vec3 true_pos(0.0, 1000.0, 0.0);
    Vec3 true_vel(200.0, 100.0, 0.0);  // 200 m/s forward, 100 m/s up
    Quaternion true_att = Quaternion::from_axis_angle(Vec3(0.0, 1.0, 0.0), 30.0 * M_PI / 180.0);
    Vec3 true_ang_vel(0.0, 0.0, 0.0);
    State6DOF true_state(true_pos, true_vel, true_att, true_ang_vel);

    std::cout << "Simulating 30 second GPS outage...\n\n";
    std::cout << "Time(s) | True X (m) | True Y (m) | Est X (m) | Est Y (m) | Pos Err (m) | Vel Err (m/s)\n";
    std::cout << "--------+------------+------------+------------+------------+-------------+---------------\n";

    const double dt = 0.01;  // 100 Hz IMU
    const int outage_duration = 30;  // 30 seconds

    for (int step = 0; step <= outage_duration / dt; ++step) {
        double t = step * dt;

        // Update true state (with gravity)
        Vec3 accel(0.0, -9.80665, 0.0);
        true_vel = true_vel + accel * dt;
        true_pos = true_pos + true_vel * dt;
        true_state.set_position(true_pos);
        true_state.set_velocity(true_vel);

        // IMU measurement
        IMUMeasurement z = imu.measure(true_state);
        z.timestamp = t;

        // Predict with IMU (dead reckoning)
        fusion.predict_with_imu(z, dt);

        // No GPS update!

        // Get estimate
        Vec3 est_pos = fusion.get_position();
        Vec3 est_vel = fusion.get_velocity();
        double pos_error = true_pos.distance_to(est_pos);
        double vel_error = true_vel.distance_to(est_vel);

        // Print every 5 seconds
        if (step % 500 == 0) {
            std::cout << std::setw(7) << std::fixed << std::setprecision(1) << t << " | "
                      << std::setw(10) << true_pos.x << " | "
                      << std::setw(10) << true_pos.y << " | "
                      << std::setw(10) << est_pos.x << " | "
                      << std::setw(10) << est_pos.y << " | "
                      << std::setw(11) << pos_error << " | "
                      << std::setw(13) << vel_error << "\n";
        }
    }

    std::cout << "\nFinal uncertainty after 30s GPS outage:\n";
    std::cout << "  Position: " << fusion.get_position_uncertainty() << " m\n";
    std::cout << "  Velocity: " << fusion.get_velocity_uncertainty() << " m/s\n";
    std::cout << "  GPS Available: " << (fusion.gps_available() ? "Yes" : "No") << "\n\n";
}

void test_sensor_fusion_gps_imu() {
    std::cout << "=== Full Sensor Fusion (GPS + IMU) ===\n\n";

    // Create sensors
    IMUModel imu(IMUGrade::INDUSTRIAL, 100.0);  // 100 Hz
    std::cout << imu.get_specifications() << "\n";

    // Initialize fusion
    NavState initial;
    initial.position = Vec3(0.0, 1000.0, 0.0);
    initial.velocity = Vec3(250.0, 50.0, 0.0);
    initial.attitude = Quaternion::identity();
    initial.timestamp = 0.0;

    SensorFusion fusion(initial, 10.0, 2.0);

    // True state (missile in flight)
    Vec3 true_pos(0.0, 0.0, 0.0);
    Vec3 true_vel(250.0, 150.0, 0.0);  // 250 m/s forward, 150 m/s up
    Quaternion true_att = Quaternion::identity();
    Vec3 true_ang_vel(0.0, 0.0, 50.0);  // 50 rad/s roll
    State6DOF true_state(true_pos, true_vel, true_att, true_ang_vel);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 1.0);

    std::cout << "Time(s) | True X (m) | True Y (m) | Est X (m) | Est Y (m) | Pos Err (m) | Mode\n";
    std::cout << "--------+------------+------------+------------+------------+-------------+---------\n";

    const double dt = 0.01;  // 100 Hz
    const int duration = 20;  // 20 seconds

    for (int step = 0; step <= duration / dt; ++step) {
        double t = step * dt;

        // Update true state (ballistic with gravity)
        Vec3 accel(0.0, -9.80665, 0.0);
        true_vel = true_vel + accel * dt;
        true_pos = true_pos + true_vel * dt;
        true_state.set_position(true_pos);
        true_state.set_velocity(true_vel);

        // IMU measurement (every update)
        IMUMeasurement imu_z = imu.measure(true_state);
        imu_z.timestamp = t;
        fusion.predict_with_imu(imu_z, dt);

        // GPS measurement (every 1 second)
        std::string mode = "IMU-DR";
        if (step % 100 == 0) {
            GPSMeasurement gps = GPSMeasurement::from_state(true_state);
            gps.timestamp = t;
            gps.position.x += 5.0 * noise(gen);
            gps.position.y += 5.0 * noise(gen);
            gps.position.z += 5.0 * noise(gen);
            gps.velocity.x += 0.1 * noise(gen);
            gps.velocity.y += 0.1 * noise(gen);
            gps.velocity.z += 0.1 * noise(gen);
            gps.timestamp = t;

            if (fusion.update_with_gps(gps)) {
                mode = "GPS+IMU";
            }
        }

        // Get estimate
        Vec3 est_pos = fusion.get_position();
        double pos_error = true_pos.distance_to(est_pos);

        // Print every 1 second
        if (step % 100 == 0) {
            std::cout << std::setw(7) << std::fixed << std::setprecision(1) << t << " | "
                      << std::setw(10) << true_pos.x << " | "
                      << std::setw(10) << true_pos.y << " | "
                      << std::setw(10) << est_pos.x << " | "
                      << std::setw(10) << est_pos.y << " | "
                      << std::setw(11) << pos_error << " | "
                      << std::setw(9) << mode << "\n";
        }
    }

    std::cout << "\n";
}

void test_gps_outage_recovery() {
    std::cout << "=== GPS Outage and Recovery ===\n\n";

    IMUModel imu(IMUGrade::TACTICAL, 100.0);

    // Start with good GPS lock
    NavState initial;
    initial.position = Vec3(0.0, 1000.0, 0.0);
    initial.velocity = Vec3(200.0, 0.0, 0.0);
    initial.attitude = Quaternion::identity();
    initial.timestamp = 0.0;

    SensorFusion fusion(initial, 1.0, 0.1);

    // True state
    Vec3 true_pos(0.0, 1000.0, 0.0);
    Vec3 true_vel(200.0, 0.0, 0.0);
    State6DOF true_state(true_pos, true_vel, Quaternion::identity(), Vec3::zero());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 1.0);

    std::cout << "Scenario:\n";
    std::cout << "  0-5s:    GPS available (initialization)\n";
    std::cout << "  5-15s:   GPS outage (dead reckoning)\n";
    std::cout << "  15-20s:  GPS recovered (re-acquisition)\n\n";

    std::cout << "Time(s) | Pos Err (m) | Vel Err (m/s) | GPS Status\n";
    std::cout << "--------+-------------+---------------+------------\n";

    const double dt = 0.01;
    const int total_steps = 2000;  // 20 seconds

    for (int step = 0; step <= total_steps; ++step) {
        double t = step * dt;

        // Update true state
        Vec3 accel(0.0, -9.80665, 0.0);
        true_vel = true_vel + accel * dt;
        true_pos = true_pos + true_vel * dt;
        true_state.set_position(true_pos);
        true_state.set_velocity(true_vel);

        // IMU measurement
        IMUMeasurement imu_z = imu.measure(true_state);
        fusion.predict_with_imu(imu_z, dt);

        // GPS: available 0-5s, outage 5-15s, recovered 15-20s
        bool gps_time = (t < 5.0) || (t >= 15.0);
        std::string gps_status = gps_time ? "Available" : "Outage";

        if (gps_time && step % 100 == 0) {
            GPSMeasurement gps = GPSMeasurement::from_state(true_state);
            gps.timestamp = t;
            gps.position.x += 5.0 * noise(gen);
            gps.position.y += 5.0 * noise(gen);
            gps.position.z += 5.0 * noise(gen);
            gps.velocity.x += 0.1 * noise(gen);
            gps.velocity.y += 0.1 * noise(gen);
            gps.velocity.z += 0.1 * noise(gen);
            fusion.update_with_gps(gps);
        }

        // Get estimate
        Vec3 est_pos = fusion.get_position();
        Vec3 est_vel = fusion.get_velocity();
        double pos_error = true_pos.distance_to(est_pos);
        double vel_error = true_vel.distance_to(est_vel);

        // Print every 1 second
        if (step % 100 == 0) {
            std::cout << std::setw(7) << std::fixed << std::setprecision(1) << t << " | "
                      << std::setw(11) << pos_error << " | "
                      << std::setw(13) << vel_error << " | "
                      << std::setw(12) << gps_status << "\n";
        }
    }

    std::cout << "\nFault Statistics:\n" << fusion.get_fault_statistics();
}

void test_multi_sensor_fusion() {
    std::cout << "=== Multi-Sensor Fusion (GPS + IMU + Radar) ===\n\n";

    // Create sensors
    IMUModel imu(IMUGrade::TACTICAL, 100.0);
    RadarModel radar(RadarType::FIRE_CONTROL);
    radar.set_position(Vec3(0.0, 0.0, 0.0));

    std::cout << imu.get_specifications() << "\n";
    std::cout << radar.get_specifications() << "\n\n";

    // Initialize fusion
    NavState initial;
    initial.position = Vec3(0.0, 1000.0, 0.0);
    initial.velocity = Vec3(300.0, 100.0, 0.0);
    initial.attitude = Quaternion::identity();
    initial.timestamp = 0.0;

    SensorFusion fusion(initial, 50.0, 5.0);

    // True state (fighter jet)
    Vec3 true_pos(5000.0, 3000.0, 0.0);
    Vec3 true_vel(250.0, 50.0, 100.0);
    State6DOF true_state(true_pos, true_vel, Quaternion::identity(), Vec3::zero());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 1.0);

    std::cout << "Time(s) | True X (m) | Est X (m) | Pos Err (m) | Sensors Used\n";
    std::cout << "--------+------------+------------+-------------+--------------\n";

    const double dt = 0.01;
    const int duration = 10;

    for (int step = 0; step <= duration / dt; ++step) {
        double t = step * dt;

        // Update true state
        Vec3 accel(0.0, -9.80665, 0.0);
        true_vel = true_vel + accel * dt;
        true_pos = true_pos + true_vel * dt;
        true_state.set_position(true_pos);
        true_state.set_velocity(true_vel);

        // IMU measurement (always available)
        IMUMeasurement imu_z = imu.measure(true_state);
        fusion.predict_with_imu(imu_z, dt);

        // GPS measurement (1 Hz)
        bool gps_used = false;
        if (step % 100 == 0) {
            GPSMeasurement gps = GPSMeasurement::from_state(true_state);
            gps.timestamp = t;
            gps.position.x += 5.0 * noise(gen);
            gps.position.y += 5.0 * noise(gen);
            gps.position.z += 5.0 * noise(gen);
            gps.velocity.x += 0.1 * noise(gen);
            gps.velocity.y += 0.1 * noise(gen);
            gps.velocity.z += 0.1 * noise(gen);
            gps_used = fusion.update_with_gps(gps);
        }

        // Radar measurement (10 Hz)
        bool radar_used = false;
        if (step % 10 == 0) {
            auto radar_z = radar.measure(true_state);
            radar_used = fusion.update_with_radar(radar_z, Vec3(0.0, 0.0, 0.0));
        }

        // Get estimate
        Vec3 est_pos = fusion.get_position();
        double pos_error = true_pos.distance_to(est_pos);

        // Print every 1 second
        if (step % 100 == 0) {
            std::string sensors = "IMU";
            if (gps_used) sensors += "+GPS";
            if (radar_used) sensors += "+Radar";

            std::cout << std::setw(7) << std::fixed << std::setprecision(1) << t << " | "
                      << std::setw(10) << true_pos.x << " | "
                      << std::setw(10) << est_pos.x << " | "
                      << std::setw(11) << pos_error << " | "
                      << std::setw(14) << sensors << "\n";
        }
    }

    std::cout << "\n";
}

void test_fault_detection() {
    std::cout << "=== Fault Detection and Rejection ===\n\n";

    IMUModel imu(IMUGrade::INDUSTRIAL, 100.0);

    NavState initial;
    initial.position = Vec3(0.0, 1000.0, 0.0);
    initial.velocity = Vec3(200.0, 0.0, 0.0);
    initial.attitude = Quaternion::identity();
    initial.timestamp = 0.0;

    SensorFusion fusion(initial, 10.0, 2.0);

    // True state
    Vec3 true_pos(0.0, 1000.0, 0.0);
    Vec3 true_vel(200.0, 0.0, 0.0);
    State6DOF true_state(true_pos, true_vel, Quaternion::identity(), Vec3::zero());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 1.0);
    std::uniform_real_distribution<> fault_trigger(0.0, 1.0);

    std::cout << "Simulating occasional GPS faults...\n\n";
    std::cout << "Time(s) | Pos Err (m) | GPS Update | Fault Detected\n";
    std::cout << "--------+-------------+------------+---------------\n";

    const double dt = 0.01;
    const int duration = 20;

    for (int step = 0; step <= duration / dt; ++step) {
        double t = step * dt;

        // Update true state
        Vec3 accel(0.0, -9.80665, 0.0);
        true_vel = true_vel + accel * dt;
        true_pos = true_pos + true_vel * dt;
        true_state.set_position(true_pos);
        true_state.set_velocity(true_vel);

        // IMU prediction
        IMUMeasurement imu_z = imu.measure(true_state);
        fusion.predict_with_imu(imu_z, dt);

        // GPS with occasional fault
        bool gps_updated = false;
        bool fault_detected = false;

        if (step % 100 == 0) {
            GPSMeasurement gps = GPSMeasurement::from_state(true_state);
            gps.timestamp = t;

            // Inject fault 20% of the time
            if (fault_trigger(gen) < 0.2) {
                // Large position error (multipath or satellite switch)
                gps.position.x += 200.0;  // 200m fault!
            } else {
                gps.position.x += 5.0 * noise(gen);
                gps.position.y += 5.0 * noise(gen);
                gps.position.z += 5.0 * noise(gen);
            }

            gps_updated = true;
            fault_detected = !fusion.update_with_gps(gps);  // Returns false if rejected
        }

        // Print every 1 second
        if (step % 100 == 0) {
            Vec3 est_pos = fusion.get_position();
            double pos_error = true_pos.distance_to(est_pos);

            std::cout << std::setw(7) << std::fixed << std::setprecision(1) << t << " | "
                      << std::setw(11) << pos_error << " | "
                      << std::setw(12) << (gps_updated ? "Yes" : "No") << " | "
                      << std::setw(13) << (fault_detected ? "YES" : "No") << "\n";
        }
    }

    std::cout << "\nFinal Statistics:\n";
    std::cout << "  Total Rejected: " << fusion.get_rejected_count() << "\n";
    std::cout << "  Filter Healthy: " << (fusion.is_healthy() ? "Yes" : "No") << "\n\n";
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║            Sensor Fusion Test Suite                          ║\n";
    std::cout << "║       GPS + IMU + Radar Integration with EKF                 ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

    test_gps_only_fusion();
    test_imu_dead_reckoning();
    test_sensor_fusion_gps_imu();
    test_gps_outage_recovery();
    test_multi_sensor_fusion();
    test_fault_detection();

    std::cout << "=== All Sensor Fusion Tests Complete ===\n";
    return 0;
}
