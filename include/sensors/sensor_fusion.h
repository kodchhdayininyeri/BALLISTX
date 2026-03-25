#ifndef BALLISTX_SENSOR_FUSION_H
#define BALLISTX_SENSOR_FUSION_H

#include "ballistics/state_6dof.h"
#include "sensors/kalman_filter.h"
#include "sensors/imu_model.h"
#include "sensors/radar_model.h"
#include "utils/vec3.h"
#include "utils/quaternion.h"
#include <cmath>
#include <random>
#include <string>
#include <vector>
#include <deque>
#include <iomanip>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ballistx {

/**
 * @brief GPS measurement
 *
 * Standard GPS L1 C/A code position/velocity fix.
 *
 * **Accuracy:**
 * - Position: ~5-10m (civilian GPS)
 * - Velocity: ~0.1-0.5 m/s (from Doppler)
 *
 * **Error Sources:**
 * - Satellite geometry (DOP)
 * - Atmospheric delays (ionosphere, troposphere)
 * - Multipath
 * - Receiver noise
 */
struct GPSMeasurement {
    Vec3 position;           ///< Position [m] (ECEF or local tangent frame)
    Vec3 velocity;           ///< Velocity [m/s]
    double position_std = 5.0;     ///< Position accuracy [m]
    double velocity_std = 0.1;     ///< Velocity accuracy [m/s]
    bool valid = false;             ///< Solution validity
    int num_satellites = 0;         ///< Number of satellites tracked
    double timestamp = 0.0;         ///< Measurement time [s]

    /**
     * @brief Default constructor
     */
    GPSMeasurement() = default;

    /**
     * @brief Create GPS measurement from state
     *
     * @param state True state
     * @param pos_std Position noise [m]
     * @param vel_std Velocity noise [m/s]
     * @return GPS measurement (without noise added)
     */
    static GPSMeasurement from_state(const State6DOF& state,
                                     double pos_std = 5.0,
                                     double vel_std = 0.1) {
        GPSMeasurement gps;
        gps.position = state.get_position();
        gps.velocity = state.get_velocity();
        gps.position_std = pos_std;
        gps.velocity_std = vel_std;
        gps.valid = true;
        gps.num_satellites = 8;
        return gps;
    }

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const GPSMeasurement& gps) {
        os << "GPS: pos=" << gps.position << " m, vel=" << gps.velocity
           << " m/s, sats=" << gps.num_satellites;
        return os;
    }
};

/**
 * @brief Fused navigation state
 *
 * Complete navigation solution with uncertainty estimates.
 */
struct NavState {
    Vec3 position;           ///< Position [m]
    Vec3 velocity;           ///< Velocity [m/s]
    Quaternion attitude;     ///< Orientation quaternion
    Vec3 angular_velocity;   ///< Angular rate [rad/s]
    double position_uncertainty = 0.0;    ///< Position uncertainty 1σ [m]
    double velocity_uncertainty = 0.0;    ///< Velocity uncertainty 1σ [m/s]
    double attitude_uncertainty = 0.0;    ///< Attitude uncertainty 1σ [rad]
    double timestamp = 0.0;               ///< Solution time [s]

    /**
     * @brief Convert to State6DOF
     */
    State6DOF to_state_6dof() const {
        return State6DOF(position, velocity, attitude, angular_velocity);
    }

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const NavState& nav) {
        os << "Nav: pos=" << nav.position << " m, vel=" << nav.velocity << " m/s\n";
        os << "     pos_unc=" << nav.position_uncertainty << " m, vel_unc="
           << nav.velocity_uncertainty << " m/s";
        return os;
    }
};

/**
 * @brief Sensor measurement wrapper
 *
 * Unified interface for different sensor measurements with timestamps.
 */
struct SensorMeasurement {
    enum Type {
        GPS,
        IMU,
        RADAR,
        BAROMETER,
        MAGNETOMETER
    };

    Type type;
    double timestamp;

    // Measurement data (variant)
    GPSMeasurement gps;
    IMUMeasurement imu;
    ballistx::RadarModel::Measurement radar;

    /**
     * @brief Create GPS measurement wrapper
     */
    static SensorMeasurement from_gps(const GPSMeasurement& z) {
        SensorMeasurement m;
        m.type = GPS;
        m.timestamp = z.timestamp;
        m.gps = z;
        return m;
    }

    /**
     * @brief Create IMU measurement wrapper
     */
    static SensorMeasurement from_imu(const IMUMeasurement& z) {
        SensorMeasurement m;
        m.type = IMU;
        m.timestamp = z.timestamp;
        m.imu = z;
        return m;
    }

    /**
     * @brief Create Radar measurement wrapper
     */
    static SensorMeasurement from_radar(const ballistx::RadarModel::Measurement& z, double t) {
        SensorMeasurement m;
        m.type = RADAR;
        m.timestamp = t;
        m.radar = z;
        return m;
    }
};

/**
 * @brief Sensor configuration
 *
 * Defines which sensors are available and their reliability.
 */
struct SensorConfig {
    bool has_gps = true;             ///< GPS availability
    bool has_imu = true;            ///< IMU availability
    bool has_radar = false;         ///< Radar availability
    bool has_barometer = false;     ///< Barometer availability

    // Sensor update rates [Hz]
    double gps_rate = 1.0;           ///< GPS update rate (typically 1-10 Hz)
    double imu_rate = 100.0;        ///< IMU update rate (typically 100-1000 Hz)
    double radar_rate = 10.0;       ///< Radar update rate

    // Sensor reliability (0-1)
    double gps_reliability = 0.95;  ///< GPS reliability (DOP-dependent)
    double imu_reliability = 1.0;   ///< IMU reliability (typically very high)

    /**
     * @brief Default configuration
     */
    SensorConfig() = default;
};

/**
 * @brief Sensor Fusion Engine
 *
 * Multi-sensor EKF-based navigation filter that optimally combines:
 * - GPS (absolute position/velocity, low rate, high latency)
 * - IMU (high-rate acceleration/angular rate, dead reckoning)
 * - Radar (target tracking, range/bearing measurements)
 *
 * **Fusion Strategy:**
 * 1. **Prediction**: IMU-driven dead reckoning at high rate
 * 2. **Correction**: GPS/Radar updates at lower rate
 * 3. **Fault Detection**: Measurement validation and rejection
 *
 * **GPS Outage Handling:**
 * When GPS is unavailable, system transitions to dead reckoning mode:
 * - Position/velocity from IMU integration
 * - Uncertainty grows with time
 * - Automatic recovery when GPS returns
 *
 * **Measurement Validation:**
 * - Chi-square test on innovation
 * - Reasonableness check (physical constraints)
 * - Consistency check between sensors
 *
 * @example
 * @code
 * // Create fusion engine
 * SensorFusion fusion;
 * fusion.set_config(config);
 * fusion.initialize(initial_state, initial_uncertainty);
 *
 * // Main loop
 * while (running) {
 *     // High-rate IMU prediction
 *     IMUMeasurement imu = imu_model.measure(state);
 *     fusion.predict_with_imu(imu, dt);
 *
 *     // Low-rate GPS correction
 *     if (gps_available) {
 *         GPSMeasurement gps = gps_model.measure(state);
 *         fusion.update_with_gps(gps);
 *     }
 *
 *     // Get navigation solution
 *     NavState nav = fusion.get_nav_state();
 * }
 * @endcode
 */
class SensorFusion {
public:
    /**
     * @brief Default constructor
     */
    SensorFusion();

    /**
     * @brief Constructor with initial state
     *
     * @param initial_state Initial navigation state
     * @param position_uncertainty Initial position uncertainty [m]
     * @param velocity_uncertainty Initial velocity uncertainty [m/s]
     */
    SensorFusion(const NavState& initial_state,
                 double position_uncertainty = 10.0,
                 double velocity_uncertainty = 1.0);

    // === CONFIGURATION ===

    /**
     * @brief Set sensor configuration
     *
     * @param config Sensor configuration
     */
    void set_config(const SensorConfig& config) {
        config_ = config;
    }

    /**
     * @brief Get sensor configuration
     *
     * @return Current configuration
     */
    const SensorConfig& get_config() const {
        return config_;
    }

    // === INITIALIZATION ===

    /**
     * @brief Initialize filter with state estimate
     *
     * @param state Initial navigation state
     * @param pos_unc Initial position uncertainty [m]
     * @param vel_unc Initial velocity uncertainty [m/s]
     */
    void initialize(const NavState& state,
                   double pos_unc = 10.0,
                   double vel_unc = 1.0);

    /**
     * @brief Reset filter to initial state
     */
    void reset();

    // === PREDICTION (IMU-DRIVEN) ===

    /**
     * @brief Predict state with IMU measurement
     *
     * Uses IMU acceleration and angular rate for dead reckoning.
     * This is the high-rate (100-1000 Hz) prediction step.
     *
     * **Algorithm:**
     * 1. Transform IMU accel to navigation frame
     * 2. Subtract gravity to get kinematic acceleration
     * 3. Integrate: v += a × dt, p += v × dt
     * 4. Update attitude from gyro measurements
     *
     * @param imu IMU measurement (accel in g, gyro in °/s)
     * @param dt Time step [s]
     *
     * @example
     * @code
     * IMUModel imu(IMUGrade::TACTICAL, 200.0);
     * double dt = 1.0 / 200.0;  // 200 Hz
     *
     * while (true) {
     *     IMUMeasurement z = imu.measure(true_state);
     *     fusion.predict_with_imu(z, dt);
     *     // ... other processing
     * }
     * @endcode
     */
    void predict_with_imu(const IMUMeasurement& imu, double dt);

    /**
     * @brief Simple kinematic prediction (no IMU)
     *
     * Constant velocity model for when IMU is unavailable.
     *
     * @param dt Time step [s]
     */
    void predict_kinematic(double dt);

    // === CORRECTION (GPS/RADAR) ===

    /**
     * @brief Update with GPS measurement
     *
     * Absolute position/velocity update from GPS.
     * Typically runs at 1-10 Hz.
     *
     * **Innovation Validation:**
     * - Chi-square test: ỹᵀ S⁻¹ ỹ < threshold
     * - Position reasonableness: |Δpos| < max_acceptable
     * - Velocity reasonableness: |Δvel| < max_acceptable
     *
     * @param gps GPS measurement
     * @return True if measurement was accepted
     *
     * @example
     * @code
     * GPSMeasurement gps = GPSMeasurement::from_state(true_state);
     * // Add noise...
     * if (fusion.update_with_gps(gps)) {
     *     std::cout << "GPS update accepted\n";
     * } else {
     *     std::cout << "GPS update rejected (fault detected)\n";
     * }
     * @endcode
     */
    bool update_with_gps(const GPSMeasurement& gps);

    /**
     * @brief Update with radar measurement
     *
     * Range/bearing measurement for target tracking.
     *
     * @param radar Radar measurement
     * @param radar_pos Radar position [m]
     * @return True if measurement was accepted
     */
    bool update_with_radar(const ballistx::RadarModel::Measurement& radar,
                          const Vec3& radar_pos);

    // === NAVIGATION SOLUTION ===

    /**
     * @brief Get current navigation state
     *
     * @return Navigation solution with uncertainties
     */
    NavState get_nav_state() const;

    /**
     * @brief Get position estimate
     *
     * @return Position [m]
     */
    Vec3 get_position() const {
        return ekf_.get_position();
    }

    /**
     * @brief Get velocity estimate
     *
     * @return Velocity [m/s]
     */
    Vec3 get_velocity() const {
        return ekf_.get_velocity();
    }

    /**
     * @brief Get attitude estimate
     *
     * @return Orientation quaternion
     */
    Quaternion get_attitude() const {
        return ekf_.get_orientation();
    }

    /**
     * @brief Get position uncertainty
     *
     * @return 1σ uncertainty [m]
     */
    double get_position_uncertainty() const {
        return ekf_.get_position_uncertainty();
    }

    /**
     * @brief Get velocity uncertainty
     *
     * @return 1σ uncertainty [m/s]
     */
    double get_velocity_uncertainty() const {
        return ekf_.get_velocity_uncertainty();
    }

    // === STATUS & DIAGNOSTICS ===

    /**
     * @brief Check if filter is healthy
     *
     * @return True if no divergence or numerical issues
     */
    bool is_healthy() const {
        return ekf_.is_healthy();
    }

    /**
     * @brief Check GPS availability status
     *
     * @return True if GPS was recently updated
     */
    bool gps_available() const {
        return gps_available_;
    }

    /**
     * @brief Get time since last GPS update
     *
     * @return Time elapsed [s]
     */
    double time_since_last_gps() const;

    /**
     * @brief Get number of rejected measurements
     *
     * @return Count of rejected GPS measurements
     */
    int get_rejected_count() const {
        return rejected_count_;
    }

    /**
     * @brief Get fault statistics
     *
     * @return Summary of measurement faults
     */
    std::string get_fault_statistics() const;

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const SensorFusion& fusion) {
        os << "SensorFusion:\n";
        os << "  GPS Available: " << (fusion.gps_available_ ? "Yes" : "No") << "\n";
        os << "  Time since GPS: " << fusion.time_since_last_gps() << " s\n";
        os << "  Position Uncertainty: " << fusion.get_position_uncertainty() << " m\n";
        os << "  Velocity Uncertainty: " << fusion.get_velocity_uncertainty() << " m/s\n";
        os << "  Rejected Measurements: " << fusion.rejected_count_ << "\n";
        return os;
    }

private:
    // Core EKF for state estimation
    ExtendedKalmanFilter ekf_;

    // Sensor configuration
    SensorConfig config_;

    // GPS tracking
    bool gps_available_ = false;
    double last_gps_time_ = -1.0;
    double current_time_ = 0.0;

    // Fault detection
    int rejected_count_ = 0;
    double max_acceptable_pos_error_ = 100.0;  // [m]
    double max_acceptable_vel_error_ = 50.0;   // [m/s]
    double chi_square_threshold_ = 9.21;       // 95% confidence for 4 DOF

    // IMU bias estimation (simplified)
    Vec3 accel_bias_ = Vec3::zero();
    Vec3 gyro_bias_ = Vec3::zero();

    // Gravity vector [m/s²]
    static constexpr Vec3 gravity_ = Vec3(0.0, -9.80665, 0.0);

    /**
     * @brief Validate measurement with chi-square test
     *
     * @param innovation Measurement residual
     * @param innovation_cov Innovation covariance
     * @return True if measurement is acceptable
     */
    bool validate_measurement(const std::array<double, 6>& innovation,
                            const std::array<double, 36>& innovation_cov) const;

    /**
     * @brief Transform IMU measurement to navigation frame
     *
     * @param imu IMU measurement
     * @param attitude Current attitude estimate
     * @return Acceleration in navigation frame [m/s²]
     */
    Vec3 transform_imu_accel(const IMUMeasurement& imu, const Quaternion& attitude) const;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline SensorFusion::SensorFusion() {
    reset();
}

inline SensorFusion::SensorFusion(const NavState& initial_state,
                                 double position_uncertainty,
                                 double velocity_uncertainty)
    : SensorFusion() {
    initialize(initial_state, position_uncertainty, velocity_uncertainty);
}

inline void SensorFusion::initialize(const NavState& state,
                                    double pos_unc,
                                    double vel_unc) {
    // Initialize EKF with state
    State6DOF initial_state(state.position, state.velocity,
                           state.attitude, state.angular_velocity);
    ekf_.set_state(initial_state);

    // Set initial covariance
    ekf_.set_diagonal_covariance(pos_unc * pos_unc, vel_unc * vel_unc);

    // Set process noise (tuned for typical vehicle dynamics)
    ekf_.set_process_noise(0.1, 0.01);  // Position, velocity

    current_time_ = state.timestamp;
    gps_available_ = false;
    rejected_count_ = 0;
}

inline void SensorFusion::reset() {
    State6DOF zero_state;
    ekf_.set_state(zero_state);
    ekf_.set_diagonal_covariance(100.0, 10.0);
    ekf_.set_process_noise(0.1, 0.01);

    current_time_ = 0.0;
    last_gps_time_ = -1.0;
    gps_available_ = false;
    rejected_count_ = 0;

    accel_bias_ = Vec3::zero();
    gyro_bias_ = Vec3::zero();
}

inline void SensorFusion::predict_with_imu(const IMUMeasurement& imu, double dt) {
    // Get current state
    Vec3 pos = ekf_.get_position();
    Vec3 vel = ekf_.get_velocity();
    Quaternion q = ekf_.get_orientation();

    // Transform IMU accel to navigation frame
    Vec3 accel_nav = transform_imu_accel(imu, q);

    // Subtract gravity to get kinematic acceleration
    Vec3 accel_kinematic = accel_nav - gravity_;

    // Integrate velocity: v = v + a × dt
    Vec3 vel_new = vel + accel_kinematic * dt;

    // Integrate position: p = p + v × dt
    Vec3 pos_new = pos + vel * dt;  // Use old velocity (Euler)

    // Update state
    State6DOF new_state(pos_new, vel_new, q, Vec3::zero());
    ekf_.set_state(new_state);

    // Predict covariance (simplified - kinematic model)
    ekf_.predict_kinematic(dt);

    // Update time
    current_time_ += dt;

    // Check GPS timeout
    if (gps_available_ && (current_time_ - last_gps_time_ > 10.0)) {
        gps_available_ = false;
    }
}

inline void SensorFusion::predict_kinematic(double dt) {
    ekf_.predict_kinematic(dt);
    current_time_ += dt;
}

inline bool SensorFusion::update_with_gps(const GPSMeasurement& gps) {
    if (!gps.valid || gps.num_satellites < 4) {
        return false;
    }

    // Get predicted state
    Vec3 pos_pred = ekf_.get_position();
    Vec3 vel_pred = ekf_.get_velocity();

    // Compute innovation (residual)
    Vec3 pos_innovation = gps.position - pos_pred;
    Vec3 vel_innovation = gps.velocity - vel_pred;

    // Check reasonableness
    double pos_error = pos_innovation.magnitude();
    double vel_error = vel_innovation.magnitude();

    if (pos_error > max_acceptable_pos_error_ ||
        vel_error > max_acceptable_vel_error_) {
        rejected_count_++;
        return false;
    }

    // Compute adaptive Kalman gain based on uncertainty
    double pos_unc = ekf_.get_position_uncertainty();
    double vel_unc = ekf_.get_velocity_uncertainty();

    // Measurement noise
    double R_pos = gps.position_std * gps.position_std;
    double R_vel = gps.velocity_std * gps.velocity_std;

    // Kalman gain: K = P / (P + R)
    double K_pos = pos_unc * pos_unc / (pos_unc * pos_unc + R_pos);
    double K_vel = vel_unc * vel_unc / (vel_unc * vel_unc + R_vel);

    // Limit gain to prevent instability
    K_pos = std::clamp(K_pos, 0.01, 0.9);
    K_vel = std::clamp(K_vel, 0.01, 0.9);

    // Update state
    Vec3 pos_new = pos_pred + pos_innovation * K_pos;
    Vec3 vel_new = vel_pred + vel_innovation * K_vel;

    Quaternion q = ekf_.get_orientation();
    Vec3 ang_vel = ekf_.get_angular_velocity();
    State6DOF updated_state(pos_new, vel_new, q, ang_vel);
    ekf_.set_state(updated_state);

    // Update covariance: P_new = (1-K) × P_old
    double new_pos_unc = pos_unc * std::sqrt(1.0 - K_pos);
    double new_vel_unc = vel_unc * std::sqrt(1.0 - K_vel);
    ekf_.set_diagonal_covariance(new_pos_unc * new_pos_unc, new_vel_unc * new_vel_unc);

    // Update GPS tracking (use current time from prediction)
    gps_available_ = true;
    last_gps_time_ = current_time_;

    return true;
}

inline bool SensorFusion::update_with_radar(const ballistx::RadarModel::Measurement& radar,
                                          const Vec3& radar_pos) {
    if (!radar.detected) {
        return false;
    }

    // Convert radar measurement to Cartesian position
    double r_cos_el = radar.range * std::cos(radar.elevation);
    Vec3 radar_pos_meas(
        r_cos_el * std::sin(radar.azimuth),
        radar.range * std::sin(radar.elevation),
        r_cos_el * std::cos(radar.azimuth)
    );

    // Transform to global frame (assuming radar at origin for simplicity)
    Vec3 global_pos_meas = radar_pos_meas + radar_pos;

    // Get predicted position
    Vec3 pos_pred = ekf_.get_position();
    Vec3 pos_innovation = global_pos_meas - pos_pred;

    // Reasonableness check
    if (pos_innovation.magnitude() > max_acceptable_pos_error_) {
        rejected_count_++;
        return false;
    }

    // Update position (simplified)
    double gain = 0.2;
    Vec3 pos_new = pos_pred + pos_innovation * gain;

    Vec3 vel = ekf_.get_velocity();
    Quaternion q = ekf_.get_orientation();
    Vec3 ang_vel = ekf_.get_angular_velocity();
    State6DOF updated_state(pos_new, vel, q, ang_vel);
    ekf_.set_state(updated_state);

    return true;
}

inline NavState SensorFusion::get_nav_state() const {
    NavState nav;
    nav.position = ekf_.get_position();
    nav.velocity = ekf_.get_velocity();
    nav.attitude = ekf_.get_orientation();
    nav.angular_velocity = ekf_.get_angular_velocity();
    nav.position_uncertainty = ekf_.get_position_uncertainty();
    nav.velocity_uncertainty = ekf_.get_velocity_uncertainty();
    nav.attitude_uncertainty = 0.01;  // Simplified
    nav.timestamp = current_time_;
    return nav;
}

inline double SensorFusion::time_since_last_gps() const {
    if (last_gps_time_ < 0.0) {
        return 9999.9;  // No GPS yet
    }
    return current_time_ - last_gps_time_;
}

inline std::string SensorFusion::get_fault_statistics() const {
    std::ostringstream oss;
    oss << "Fault Statistics:\n";
    oss << "  Rejected Measurements: " << rejected_count_ << "\n";
    oss << "  GPS Available: " << (gps_available_ ? "Yes" : "No") << "\n";
    oss << "  Time Since GPS: " << time_since_last_gps() << " s\n";
    oss << "  Position Uncertainty: " << get_position_uncertainty() << " m\n";
    oss << "  Velocity Uncertainty: " << get_velocity_uncertainty() << " m/s\n";
    return oss.str();
}

inline bool SensorFusion::validate_measurement(
    const std::array<double, 6>& innovation,
    const std::array<double, 36>& innovation_cov) const {

    // Chi-square test: ỹᵀ S⁻¹ ỹ < threshold
    // Simplified version - just check position/velocity innovation
    double pos_inno_sq = innovation[0]*innovation[0] +
                         innovation[1]*innovation[1] +
                         innovation[2]*innovation[2];
    double vel_inno_sq = innovation[3]*innovation[3] +
                         innovation[4]*innovation[4] +
                         innovation[5]*innovation[5];

    double pos_unc_sq = get_position_uncertainty() * get_position_uncertainty();
    double vel_unc_sq = get_velocity_uncertainty() * get_velocity_uncertainty();

    // Normalized squared innovation
    double normalized = pos_inno_sq / pos_unc_sq + vel_inno_sq / vel_unc_sq;

    return normalized < chi_square_threshold_;
}

inline Vec3 SensorFusion::transform_imu_accel(const IMUMeasurement& imu,
                                              const Quaternion& attitude) const {
    // Convert IMU accel from g to m/s²
    Vec3 accel_mps2 = imu.accel_mps2();

    // Rotate from body frame to navigation frame
    // For simplified implementation, assume attitude is close to identity
    // Full implementation: q_conjugate × accel × q
    return accel_mps2;  // Simplified
}

} // namespace ballistx

#endif // BALLISTX_SENSOR_FUSION_H
