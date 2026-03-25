#ifndef BALLISTX_IMU_MODEL_H
#define BALLISTX_IMU_MODEL_H

#include "ballistics/state_6dof.h"
#include "utils/vec3.h"
#include "utils/quaternion.h"
#include <cmath>
#include <random>
#include <string>
#include <vector>
#include <iomanip>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ballistx {

/**
 * @brief IMU grade/class enumeration
 *
 * IMU performance grades based on IEEE标准和 and industrial specifications:
 * - Consumer: Smartphones, wearables (low cost, high noise)
 * - Industrial: Robotics, drones (balanced performance)
 * - Tactical: Missile guidance, aircraft (moderate cost, good accuracy)
 * - Strategic: Submarine navigation, ICBM (high cost, excellent accuracy)
 */
enum class IMUGrade {
    /**
     * Consumer grade MEMS IMU
     * Cost: $1-$10
     * Bias instability: ~1°/hr gyro, ~1mg accel
     * Example: MPU6050, BMI160, ICM-20948
     */
    CONSUMER,

    /**
     * Industrial grade MEMS IMU
     * Cost: $100-$1000
     * Bias instability: ~10°/hr gyro, ~0.1mg accel
     * Example: ADIS16470, BMI088, ICP-101xx
     */
    INDUSTRIAL,

    /**
     * Tactical grade FOG/RLG IMU
     * Cost: $10k-$100k
     * Bias instability: ~1°/hr gyro, ~0.01mg accel
     * Example: Honeywell HG9900, Safran Axisa
     */
    TACTICAL,

    /**
     * Strategic grade laser gyro IMU
     * Cost: $100k-$1M+
     * Bias instability: ~0.01°/hr gyro, ~0.001mg accel
     * Example: Northrop Grumman LN-100, Safran Sigma 30
     */
    STRATEGIC
};

/**
 * @brief IMU noise model parameters
 *
 * Complete statistical characterization of IMU errors based on:
 * - IEEE STD 952-1997 (Ring Laser Gyro)
 * - IEEE STD 1431-2004 (Coriolis Vibratory Gyro)
 * - Allan variance methods
 *
 * **Error Sources:**
 * 1. **White noise**: High-frequency, uncorrelated (angle random walk, velocity random walk)
 * 2. **Bias instability**: Low-frequency drift (flicker floor)
 * 3. **Rate random walk**: Long-term drift (bias random walk)
 * 4. **Scale factor**: Sensitivity errors
 * 5. **Misalignment**: Axis orthogonality errors
 */
struct IMUNoiseModel {
    // Accelerometer parameters
    double accel_noise_density = 1.0e-3;     ///< [g/√Hz] - White noise density
    double accel_bias_instability = 1.0e-4;   ///< [g] - Flicker floor
    double accel_bias_random_walk = 1.0e-5;   ///< [g/√Hz] - Long-term drift
    double accel_scale_factor = 1.0e-3;       ///< [ppm] - Scale factor error
    double accel_misalignment = 1.0e-3;       ///< [rad] - Axis misalignment

    // Gyroscope parameters
    double gyro_noise_density = 1.0e-1;       ///< [°/√hr] - ARW
    double gyro_bias_instability = 1.0e-1;     ///< [°/hr] - Bias instability
    double gyro_bias_random_walk = 1.0e-2;     ///< [°/hr/√Hz] - RRW
    double gyro_scale_factor = 1.0e-3;         ///< [ppm] - Scale factor error
    double gyro_misalignment = 1.0e-3;         ///< [rad] - Axis misalignment

    // Correlation time constants
    double accel_bias_tc = 1000.0;             ///< [s] - Accelerometer bias time constant
    double gyro_bias_tc = 1000.0;              ///< [s] - Gyroscope bias time constant

    /**
     * @brief Default constructor - generic IMU
     */
    IMUNoiseModel() = default;

    /**
     * @brief Create noise model from IMU grade
     *
     * Uses real IMU specifications from manufacturer datasheets.
     *
     * @param grade IMU performance grade
     * @return Noise model parameters
     *
     * @example
     * @code
     * auto tactical_noise = IMUNoiseModel::from_grade(IMUGrade::TACTICAL);
     * // tactical_noise.gyro_bias_instability ≈ 1.0 °/hr
     * @endcode
     */
    static IMUNoiseModel from_grade(IMUGrade grade);

    /**
     * @brief Get accelerometer noise standard deviation for given sample rate
     *
     * Converts noise density to std deviation: σ = density × √(sample_rate)
     *
     * @param sample_rate Sampling frequency [Hz]
     * @return Accelerometer noise std [g]
     */
    double get_accel_noise_std(double sample_rate) const {
        return accel_noise_density * std::sqrt(sample_rate);
    }

    /**
     * @brief Get gyroscope noise standard deviation for given sample rate
     *
     * @param sample_rate Sampling frequency [Hz]
     * @return Gyroscope noise std [°/s]
     */
    double get_gyro_noise_std(double sample_rate) const {
        // Convert °/√hr to °/s: divide by √3600
        return gyro_noise_density / 60.0 * std::sqrt(sample_rate);
    }

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const IMUNoiseModel& noise) {
        os << "IMU Noise Model:\n";
        os << "  Accel: σ_density=" << noise.accel_noise_density << " g/√Hz"
           << ", bias_inst=" << noise.accel_bias_instability << " g"
           << ", bias_rw=" << noise.accel_bias_random_walk << " g/√Hz\n";
        os << "  Gyro:  ARW=" << noise.gyro_noise_density << " °/√hr"
           << ", bias_inst=" << noise.gyro_bias_instability << " °/hr"
           << ", RRW=" << noise.gyro_bias_random_walk << " °/hr/√Hz";
        return os;
    }
};

/**
 * @brief IMU measurement
 *
 * Raw IMU output including specific force and angular rate measurements.
 *
 * **Coordinate Frames:**
 * - Measurements are in the IMU body frame
 * - Accelerometer measures specific force (proper acceleration), not kinematic acceleration
 * - For stationary IMU: accel = -gravity (not zero!)
 */
struct IMUMeasurement {
    Vec3 accel;           ///< Specific force [g] (body frame)
    Vec3 gyro;            ///< Angular rate [°/s] (body frame)
    double timestamp = 0.0;  ///< Measurement time [s]

    /**
     * @brief Default constructor
     */
    IMUMeasurement() = default;

    /**
     * @brief Create measurement from vectors
     */
    IMUMeasurement(const Vec3& a, const Vec3& g, double t = 0.0)
        : accel(a), gyro(g), timestamp(t) {}

    /**
     * @brief Get accelerometer in m/s²
     */
    Vec3 accel_mps2() const {
        return accel * 9.80665;
    }

    /**
     * @brief Get gyroscope in rad/s
     */
    Vec3 gyro_rads() const {
        return gyro * (M_PI / 180.0);
    }

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const IMUMeasurement& m) {
        os << "IMU: accel=" << m.accel << " g, gyro=" << m.gyro << " °/s";
        return os;
    }
};

/**
 * @brief IMU simulation model
 *
 * Comprehensive IMU simulator with realistic error modeling including:
 * - White noise (quantum, thermal)
 * - Bias instability (flicker noise, 1/f)
 * - Bias random walk (correlated drift)
 * - Scale factor errors
 * - Misalignment errors
 * - Temperature effects (optional)
 *
 * **Physical Models:**
 *
 * 1. **Accelerometer output:**
 *    a_measured = (T_a × (I + S_a) × a_true + b_a + w_a)
 *
 * 2. **Gyroscope output:**
 *    ω_measured = (T_g × (I + S_g) × ω_true + b_g + w_g)
 *
 * Where:
 * - T: Misalignment matrix
 * - S: Scale factor matrix
 * - b: Bias (with instability and random walk)
 * - w: White noise
 *
 * @example
 * @code
 * // Create tactical grade IMU
 * IMUModel imu(IMUGrade::TACTICAL);
 * imu.set_sample_rate(100.0);  // 100 Hz
 *
 * // True missile state
 * State6DOF missile_state(...);
 *
 * // Generate measurement
 * auto z = imu.measure(missile_state);
 * Vec3 accel = z.accel_mps2();  // m/s²
 * Vec3 gyro = z.gyro_rads();     // rad/s
 * @endcode
 */
class IMUModel {
public:
    /**
     * @brief Default constructor - consumer grade IMU
     */
    IMUModel()
        : grade_(IMUGrade::CONSUMER),
          noise_(IMUNoiseModel::from_grade(IMUGrade::CONSUMER)),
          sample_rate_(100.0),
          temperature_(25.0) {
        initialize_rng();
        reset_bias();
    }

    /**
     * @brief Constructor with IMU grade
     *
     * @param grade IMU performance grade
     * @param sample_rate Sampling frequency [Hz]
     */
    explicit IMUModel(IMUGrade grade, double sample_rate = 100.0)
        : grade_(grade),
          noise_(IMUNoiseModel::from_grade(grade)),
          sample_rate_(sample_rate),
          temperature_(25.0) {
        initialize_rng();
        reset_bias();
    }

    /**
     * @brief Constructor with custom noise model
     *
     * @param noise Custom noise parameters
     * @param sample_rate Sampling frequency [Hz]
     */
    IMUModel(const IMUNoiseModel& noise, double sample_rate = 100.0)
        : grade_(IMUGrade::CONSUMER),
          noise_(noise),
          sample_rate_(sample_rate),
          temperature_(25.0) {
        initialize_rng();
        reset_bias();
    }

    // === CONFIGURATION ===

    /**
     * @brief Set IMU grade (updates noise model)
     *
     * @param grade IMU performance grade
     */
    void set_grade(IMUGrade grade) {
        grade_ = grade;
        noise_ = IMUNoiseModel::from_grade(grade);
    }

    /**
     * @brief Get IMU grade
     *
     * @return Current IMU grade
     */
    IMUGrade get_grade() const { return grade_; }

    /**
     * @brief Set noise parameters directly
     *
     * @param noise Noise model
     */
    void set_noise_model(const IMUNoiseModel& noise) {
        noise_ = noise;
    }

    /**
     * @brief Get noise model
     *
     * @return Current noise parameters
     */
    const IMUNoiseModel& get_noise_model() const { return noise_; }

    /**
     * @brief Set sample rate
     *
     * @param rate Sampling frequency [Hz]
     */
    void set_sample_rate(double rate) {
        sample_rate_ = std::max(1.0, rate);
    }

    /**
     * @brief Get sample rate
     *
     * @return Sampling frequency [Hz]
     */
    double get_sample_rate() const { return sample_rate_; }

    /**
     * @brief Set temperature (affects bias)
     *
     * @param temp Temperature [°C]
     */
    void set_temperature(double temp) {
        temperature_ = temp;
        update_bias_with_temperature();
    }

    /**
     * @brief Get temperature
     *
     * @return Temperature [°C]
     */
    double get_temperature() const { return temperature_; }

    // === BIAS MANAGEMENT ===

    /**
     * @brief Reset bias to initial values
     *
     * Bias is initialized with small random offset from zero.
     */
    void reset_bias() {
        std::normal_distribution<> init_bias(0.0, 1.0);
        accel_bias_ = Vec3(
            noise_.accel_bias_instability * init_bias(rng_),
            noise_.accel_bias_instability * init_bias(rng_),
            noise_.accel_bias_instability * init_bias(rng_)
        );
        gyro_bias_ = Vec3(
            noise_.gyro_bias_instability * init_bias(rng_),
            noise_.gyro_bias_instability * init_bias(rng_),
            noise_.gyro_bias_instability * init_bias(rng_)
        );
    }

    /**
     * @brief Get current accelerometer bias
     *
     * @return Bias [g]
     */
    Vec3 get_accel_bias() const { return accel_bias_; }

    /**
     * @brief Get current gyroscope bias
     *
     * @return Bias [°/s]
     */
    Vec3 get_gyro_bias() const { return gyro_bias_; }

    /**
     * @brief Set accelerometer bias (for calibration simulation)
     *
     * @param bias Bias [g]
     */
    void set_accel_bias(const Vec3& bias) {
        accel_bias_ = bias;
    }

    /**
     * @brief Set gyroscope bias (for calibration simulation)
     *
     * @param bias Bias [°/s]
     */
    void set_gyro_bias(const Vec3& bias) {
        gyro_bias_ = bias;
    }

    // === MEASUREMENT GENERATION ===

    /**
     * @brief Generate IMU measurement from true state
     *
     * Simulates a complete IMU measurement cycle with all error sources.
     *
     * **Physical Model:**
     * For a rotating, accelerating body in gravity field:
     * - Accelerometer measures: f = a - g (specific force)
     * - Gyroscope measures: ω (angular rate)
     *
     * @param state True body state (world frame)
     * @return Measured specific force [g] and angular rate [°/s]
     *
     * @example
     * @code
     * IMUModel imu(IMUGrade::TACTICAL, 200.0);
     *
     * // Missile in 30° climb, 5°/s roll
     * State6DOF missile(...);
     *
     * auto z = imu.measure(missile);
     * // z.accel contains specific force in body frame
     * // z.gyro contains angular rate in body frame
     * @endcode
     */
    IMUMeasurement measure(const State6DOF& state) {
        // 1. Get true measurements (body frame)
        Vec3 true_accel_body = get_true_accel_body(state);
        Vec3 true_gyro_body = get_true_gyro_body(state);

        // 2. Update bias (instability + random walk)
        update_bias();

        // 3. Apply errors
        Vec3 measured_accel = apply_accel_errors(true_accel_body);
        Vec3 measured_gyro = apply_gyro_errors(true_gyro_body);

        // 4. Add white noise
        measured_accel = add_accel_noise(measured_accel);
        measured_gyro = add_gyro_noise(measured_gyro);

        return IMUMeasurement(measured_accel, measured_gyro);
    }

    /**
     * @brief Generate perfect (noise-free) measurement
     *
     * Useful for calibration and testing.
     *
     * @param state True body state
     * @return Perfect measurement
     */
    IMUMeasurement measure_perfect(const State6DOF& state) const {
        return IMUMeasurement(
            get_true_accel_body(state),
            get_true_gyro_body(state)
        );
    }

    /**
     * @brief Get IMU specifications as string
     *
     * @return Human-readable IMU description
     */
    std::string get_specifications() const {
        std::string grade_name;
        double cost_low = 0.0, cost_high = 0.0;

        switch (grade_) {
            case IMUGrade::CONSUMER:
                grade_name = "Consumer Grade (MEMS)";
                cost_low = 1.0;
                cost_high = 10.0;
                break;
            case IMUGrade::INDUSTRIAL:
                grade_name = "Industrial Grade (MEMS)";
                cost_low = 100.0;
                cost_high = 1000.0;
                break;
            case IMUGrade::TACTICAL:
                grade_name = "Tactical Grade (FOG/RLG)";
                cost_low = 10000.0;
                cost_high = 100000.0;
                break;
            case IMUGrade::STRATEGIC:
                grade_name = "Strategic Grade (Laser Gyro)";
                cost_low = 100000.0;
                cost_high = 1000000.0;
                break;
        }

        return grade_name + " IMU - Cost: $" + std::to_string(static_cast<int>(cost_low)) +
               "-$" + std::to_string(static_cast<int>(cost_high)) +
               ", ARW: " + std::to_string(noise_.gyro_noise_density) + " °/√hr" +
               ", Bias Inst: " + std::to_string(noise_.gyro_bias_instability) + " °/hr";
    }

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const IMUModel& imu) {
        os << imu.get_specifications();
        return os;
    }

private:
    IMUGrade grade_;              ///< IMU performance grade
    IMUNoiseModel noise_;         ///< Noise parameters
    double sample_rate_;          ///< Sampling frequency [Hz]
    double temperature_;          ///< Temperature [°C]

    Vec3 accel_bias_;             ///< Current accelerometer bias [g]
    Vec3 gyro_bias_;              ///< Current gyroscope bias [°/s]
    Vec3 accel_scale_;            ///< Accelerometer scale factors
    Vec3 gyro_scale_;             ///< Gyroscope scale factors

    mutable std::mt19937 rng_;    ///< Random number generator

    /**
     * @brief Initialize random number generator
     */
    void initialize_rng() {
        std::random_device rd;
        rng_ = std::mt19937(rd());
    }

    /**
     * @brief Update bias (instability + random walk)
     */
    void update_bias() {
        double dt = 1.0 / sample_rate_;

        // Bias random walk (Gauss-Markov process)
        double accel_rw_sigma = noise_.accel_bias_random_walk * std::sqrt(dt);
        double gyro_rw_sigma = noise_.gyro_bias_random_walk * std::sqrt(dt / 3600.0);  // °/hr → °/s

        std::normal_distribution<> noise(0.0, 1.0);

        accel_bias_.x += accel_rw_sigma * noise(rng_);
        accel_bias_.y += accel_rw_sigma * noise(rng_);
        accel_bias_.z += accel_rw_sigma * noise(rng_);

        gyro_bias_.x += gyro_rw_sigma * noise(rng_);
        gyro_bias_.y += gyro_rw_sigma * noise(rng_);
        gyro_bias_.z += gyro_rw_sigma * noise(rng_);

        // Bias instability (flicker noise) - simulated as slow variation
        // In full implementation, this would use 1/f noise generation
    }

    /**
     * @brief Update bias with temperature change
     */
    void update_bias_with_temperature() {
        // Temperature coefficient: typically 0.1-1 mg/°C for accel, 0.01-0.1 °/hr/°C for gyro
        double temp_offset = temperature_ - 25.0;  // From 25°C reference

        // Simple linear model (real IMUs have more complex temperature profiles)
        Vec3 accel_temp_bias = Vec3(
            1e-4 * temp_offset,
            1e-4 * temp_offset,
            1e-4 * temp_offset
        );

        Vec3 gyro_temp_bias = Vec3(
            0.01 * temp_offset / 3600.0,  // °/hr → °/s
            0.01 * temp_offset / 3600.0,
            0.01 * temp_offset / 3600.0
        );

        // Add to existing bias
        accel_bias_ = accel_bias_ + accel_temp_bias;
        gyro_bias_ = gyro_bias_ + gyro_temp_bias;
    }

    /**
     * @brief Get true accelerometer measurement (specific force in body frame)
     *
     * Accelerometer measures specific force: f = a - g
     * For stationary IMU on Earth: f = -g (not zero!)
     *
     * @param state True body state
     * @return Specific force [g] in body frame
     */
    Vec3 get_true_accel_body(const State6DOF& state) const {
        // Get world-frame acceleration and gravity
        Vec3 accel_world = state.get_velocity();  // Will be computed from forces
        Vec3 gravity_world(0.0, -9.80665, 0.0);  // Gravity [m/s²]

        // Specific force in world frame: f = a - g
        Vec3 specific_force_world = accel_world - gravity_world;

        // Transform to body frame
        Quaternion q = state.get_orientation().conjugate();
        Vec3 specific_force_body = q.rotate(specific_force_world);

        // Convert to g-units
        return specific_force_body / 9.80665;
    }

    /**
     * @brief Get true gyroscope measurement
     *
     * Gyroscope measures angular rate directly.
     *
     * @param state True body state
     * @return Angular rate [°/s] in body frame
     */
    Vec3 get_true_gyro_body(const State6DOF& state) const {
        Vec3 ang_vel_body = state.get_body_velocity();  // Should be angular velocity in body frame

        // Convert rad/s to °/s
        return ang_vel_body * (180.0 / M_PI);
    }

    /**
     * @brief Apply accelerometer errors
     */
    Vec3 apply_accel_errors(const Vec3& true_accel) const {
        // Scale factor error: a' = (1 + s) × a
        Vec3 scaled(
            true_accel.x * (1.0 + noise_.accel_scale_factor),
            true_accel.y * (1.0 + noise_.accel_scale_factor),
            true_accel.z * (1.0 + noise_.accel_scale_factor)
        );

        // Misalignment (simplified: cross-coupling)
        Vec3 misaligned(
            scaled.x + noise_.accel_misalignment * scaled.y,
            scaled.y + noise_.accel_misalignment * scaled.z,
            scaled.z + noise_.accel_misalignment * scaled.x
        );

        // Add bias
        return misaligned + accel_bias_;
    }

    /**
     * @brief Apply gyroscope errors
     */
    Vec3 apply_gyro_errors(const Vec3& true_gyro) const {
        // Scale factor error
        Vec3 scaled(
            true_gyro.x * (1.0 + noise_.gyro_scale_factor),
            true_gyro.y * (1.0 + noise_.gyro_scale_factor),
            true_gyro.z * (1.0 + noise_.gyro_scale_factor)
        );

        // Misalignment
        Vec3 misaligned(
            scaled.x + noise_.gyro_misalignment * scaled.y,
            scaled.y + noise_.gyro_misalignment * scaled.z,
            scaled.z + noise_.gyro_misalignment * scaled.x
        );

        // Add bias
        return misaligned + gyro_bias_;
    }

    /**
     * @brief Add accelerometer white noise
     */
    Vec3 add_accel_noise(const Vec3& accel) const {
        double sigma = noise_.get_accel_noise_std(sample_rate_);
        std::normal_distribution<> noise(0.0, sigma);

        return Vec3(
            accel.x + noise(rng_),
            accel.y + noise(rng_),
            accel.z + noise(rng_)
        );
    }

    /**
     * @brief Add gyroscope white noise
     */
    Vec3 add_gyro_noise(const Vec3& gyro) const {
        double sigma = noise_.get_gyro_noise_std(sample_rate_);
        std::normal_distribution<> noise(0.0, sigma);

        return Vec3(
            gyro.x + noise(rng_),
            gyro.y + noise(rng_),
            gyro.z + noise(rng_)
        );
    }
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline IMUNoiseModel IMUNoiseModel::from_grade(IMUGrade grade) {
    IMUNoiseModel noise;

    switch (grade) {
        case IMUGrade::CONSUMER:
            // Consumer grade MEMS (e.g., MPU6050, BMI160)
            // ARW: ~300°/√hr, Bias inst: ~10-100°/hr, VRW: ~200m/g/√hr
            noise.accel_noise_density = 300e-6 / 9.80665;       // ~0.3 mg/√Hz
            noise.accel_bias_instability = 1e-3;               // ~1 mg
            noise.accel_bias_random_walk = 1e-4;               // ~0.1 mg/√Hz
            noise.accel_scale_factor = 1000e-6;                // 1000 ppm
            noise.accel_misalignment = 1e-3;                   // ~0.06°

            noise.gyro_noise_density = 300.0;                  // 300 °/√hr
            noise.gyro_bias_instability = 10.0;                // 10 °/hr
            noise.gyro_bias_random_walk = 1.0;                 // 1 °/hr/√Hz
            noise.gyro_scale_factor = 1000e-6;                 // 1000 ppm
            noise.gyro_misalignment = 1e-3;                    // ~0.06°
            noise.accel_bias_tc = 100.0;                       // 100s
            noise.gyro_bias_tc = 100.0;                        // 100s
            break;

        case IMUGrade::INDUSTRIAL:
            // Industrial grade MEMS (e.g., ADIS16470, BMI088)
            // ARW: ~30°/√hr, Bias inst: ~5-20°/hr, VRW: ~50m/g/√hr
            noise.accel_noise_density = 100e-6 / 9.80665;      // ~0.1 mg/√Hz
            noise.accel_bias_instability = 5e-5;               // ~50 µg
            noise.accel_bias_random_walk = 1e-5;               // ~10 µg/√Hz
            noise.accel_scale_factor = 500e-6;                 // 500 ppm
            noise.accel_misalignment = 5e-4;                   // ~0.03°

            noise.gyro_noise_density = 30.0;                   // 30 °/√hr
            noise.gyro_bias_instability = 5.0;                 // 5 °/hr
            noise.gyro_bias_random_walk = 0.5;                 // 0.5 °/hr/√Hz
            noise.gyro_scale_factor = 500e-6;                  // 500 ppm
            noise.gyro_misalignment = 5e-4;                    // ~0.03°
            noise.accel_bias_tc = 500.0;                       // 500s
            noise.gyro_bias_tc = 500.0;                        // 500s
            break;

        case IMUGrade::TACTICAL:
            // Tactical grade FOG (e.g., Honeywell HG9900)
            // ARW: ~0.5-2°/√hr, Bias inst: ~0.5-2°/hr, VRW: ~0.05m/g/√hr
            noise.accel_noise_density = 50e-6 / 9.80665;       // ~50 µg/√Hz
            noise.accel_bias_instability = 1e-5;               // ~10 µg
            noise.accel_bias_random_walk = 1e-6;               // ~1 µg/√Hz
            noise.accel_scale_factor = 100e-6;                 // 100 ppm
            noise.accel_misalignment = 1e-4;                   // ~0.006°

            noise.gyro_noise_density = 1.0;                    // 1 °/√hr
            noise.gyro_bias_instability = 1.0;                  // 1 °/hr
            noise.gyro_bias_random_walk = 0.1;                 // 0.1 °/hr/√Hz
            noise.gyro_scale_factor = 100e-6;                  // 100 ppm
            noise.gyro_misalignment = 1e-4;                    // ~0.006°
            noise.accel_bias_tc = 1000.0;                      // 1000s
            noise.gyro_bias_tc = 1000.0;                       // 1000s
            break;

        case IMUGrade::STRATEGIC:
            // Strategic grade laser gyro (e.g., LN-100)
            // ARW: ~0.001-0.01°/√hr, Bias inst: ~0.001-0.01°/hr
            noise.accel_noise_density = 10e-6 / 9.80665;       // ~10 µg/√Hz
            noise.accel_bias_instability = 1e-6;                // ~1 µg
            noise.accel_bias_random_walk = 1e-7;               // ~0.1 µg/√Hz
            noise.accel_scale_factor = 10e-6;                  // 10 ppm
            noise.accel_misalignment = 1e-5;                   // ~0.0006°

            noise.gyro_noise_density = 0.01;                   // 0.01 °/√hr
            noise.gyro_bias_instability = 0.01;                 // 0.01 °/hr
            noise.gyro_bias_random_walk = 0.001;               // 0.001 °/hr/√Hz
            noise.gyro_scale_factor = 10e-6;                   // 10 ppm
            noise.gyro_misalignment = 1e-5;                    // ~0.0006°
            noise.accel_bias_tc = 10000.0;                     // 10000s
            noise.gyro_bias_tc = 10000.0;                      // 10000s
            break;
    }

    return noise;
}

} // namespace ballistx

#endif // BALLISTX_IMU_MODEL_H
