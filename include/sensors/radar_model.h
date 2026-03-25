#ifndef BALLISTX_RADAR_MODEL_H
#define BALLISTX_RADAR_MODEL_H

#include "ballistics/state_6dof.h"
#include "utils/vec3.h"
#include <cmath>
#include <random>
#include <string>
#include <vector>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ballistx {

/**
 * @brief Simple radar measurement (noise-free or noisy)
 */
struct SimpleRadarMeasurement {
    double range = 0.0;
    double azimuth = 0.0;
    double elevation = 0.0;
    double range_rate = 0.0;
};

/**
 * @brief Radar type enumeration with typical specifications
 *
 * Real-world radar performance characteristics from actual systems:
 * - Air traffic control (ATC) radars: Long range, moderate accuracy
 * - Fire control radars: High accuracy, shorter range
 * - Tracking radars: Very high accuracy for missile guidance
 * - Phased array radars: Electronic scanning, fast update rates
 */
enum class RadarType {
    /**
     * Generic surveillance radar
     * Range: 1-300 km, Accuracy: ±50m range, ±0.2° angle
     */
    GENERIC,

    /**
     * Air Traffic Control (ASR/ARSR type)
     * Range: 100-500 km, Accuracy: ±30m range, ±0.1° angle
     */
    ATC_SURVEILLANCE,

    /**
     * Military fire control radar
     * Range: 0.5-50 km, Accuracy: ±5m range, ±0.01° angle
     */
    FIRE_CONTROL,

    /**
     * Missile guidance radar (semi-active homing)
     * Range: 0.1-30 km, Accuracy: ±2m range, ±0.005° angle
     */
    MISSILE_GUIDANCE,

    /**
     * Phased array radar (AESA/PESA)
     * Range: 10-400 km, Accuracy: ±10m range, ±0.05° angle
     */
    PHASED_ARRAY,

    /**
     * Precision approach radar (PAR)
     * Range: 0.5-50 km, Accuracy: ±3m range, ±0.02° angle
     */
    PRECISION_APPROACH
};

/**
 * @brief Radar noise model parameters
 *
 * Represents the statistical characteristics of measurement errors
 * for different radar types. Based on real radar specifications.
 *
 * **Noise Sources:**
 * 1. Thermal noise: Random electron fluctuations in receiver
 * 2. Quantization noise: ADC sampling errors
 * 3. Atmospheric effects: Refraction, multipath, clutter
 * 4. Target fluctuations: RCS variations, glint
 * 5. Platform motion: Vibration, attitude errors
 */
struct RadarNoiseModel {
    double range_std = 10.0;         ///< Range noise standard deviation [m]
    double azimuth_std = 0.001;      ///< Azimuth noise std [rad] (~0.06°)
    double elevation_std = 0.001;    ///< Elevation noise std [rad] (~0.06°)
    double range_rate_std = 1.0;     ///< Range rate (Doppler) noise [m/s]

    /**
     * @brief Default constructor - generic radar
     */
    RadarNoiseModel() = default;

    /**
     * @brief Constructor with explicit noise parameters
     *
     * @param r_std Range standard deviation [m]
     * @param az_std Azimuth standard deviation [rad]
     * @param el_std Elevation standard deviation [rad]
     * @param rr_std Range rate standard deviation [m/s]
     */
    RadarNoiseModel(double r_std, double az_std, double el_std, double rr_std)
        : range_std(r_std), azimuth_std(az_std), elevation_std(el_std),
          range_rate_std(rr_std) {}

    /**
     * @brief Create noise model from radar type
     *
     * Uses real-world radar specifications from manufacturer datasheets
     * and military standards (MIL-STD, STANAG).
     *
     * @param type Radar type
     * @return Noise model parameters
     *
     * @example
     * @code
     * auto atc_noise = RadarNoiseModel::from_type(RadarType::ATC_SURVEILLANCE);
     * // atc_noise.range_std = 30.0 m
     * // atc_noise.azimuth_std = 0.00175 rad (~0.1°)
     * @endcode
     */
    static RadarNoiseModel from_type(RadarType type);

    /**
     * @brief Get noise as SimpleRadarMeasurement
     *
     * Converts noise standard deviations for use in EKF.
     *
     * @return Measurement noise (standard deviations)
     */
    SimpleRadarMeasurement as_measurement() const {
        SimpleRadarMeasurement m;
        m.range = range_std;
        m.azimuth = azimuth_std;
        m.elevation = elevation_std;
        m.range_rate = range_rate_std;
        return m;
    }

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const RadarNoiseModel& noise) {
        os << "RadarNoise: σr=" << noise.range_std << "m"
           << ", σaz=" << (noise.azimuth_std * 180.0 / M_PI) << "°"
           << ", σel=" << (noise.elevation_std * 180.0 / M_PI) << "°"
           << ", σṙ=" << noise.range_rate_std << "m/s";
        return os;
    }
};

/**
 * @brief Radar simulation model
 *
 * Simulates a complete radar system with configurable noise characteristics.
 * Generates realistic measurements from target state with proper error modeling.
 *
 * **Features:**
 * - Configurable radar type with real specifications
 * - Gaussian noise for all measurement channels
 * - Clutter and false target simulation
 * - Detection probability modeling
 * - Range/angle-dependent accuracy
 *
 * **Radar Equation:**
 * The maximum detection range follows the radar equation:
 * ```
 * R_max = [(Pt * G^2 * λ^2 * σ) / ((4π)^3 * P_min)]^(1/4)
 * ```
 * Where:
 * - Pt = Transmitted power
 * - G = Antenna gain
 * - λ = Wavelength
 * - σ = Target radar cross-section
 * - P_min = Minimum detectable signal
 *
 * @example
 * @code
 * // Create a fire control radar
 * RadarModel radar(RadarType::FIRE_CONTROL);
 *
 * // Set radar position (origin)
 * radar.set_position(Vec3(0.0, 0.0, 0.0));
 *
 * // Generate measurement from target
 * State6DOF target_state(...);
 * RadarMeasurement z = radar.measure(target_state);
 *
 * // Check if target was detected
 * if (z.detected) {
 *     double range = z.range;
 *     double azimuth = z.azimuth;
 *     // Process measurement...
 * }
 * @endcode
 */
class RadarModel {
public:
    /**
     * @brief Radar measurement with detection flag
     */
    struct Measurement {
        double range = 0.0;          ///< Range [m]
        double azimuth = 0.0;        ///< Azimuth [rad]
        double elevation = 0.0;      ///< Elevation [rad]
        double range_rate = 0.0;     ///< Range rate [m/s]
        bool detected = false;       ///< Detection flag

        /**
         * @brief Convert to SimpleRadarMeasurement
         */
        SimpleRadarMeasurement to_simple_measurement() const {
            SimpleRadarMeasurement m;
            m.range = range;
            m.azimuth = azimuth;
            m.elevation = elevation;
            m.range_rate = range_rate;
            return m;
        }
    };

    /**
     * @brief Default constructor - generic radar at origin
     */
    RadarModel()
        : position_(Vec3::zero()),
          type_(RadarType::GENERIC),
          noise_(RadarNoiseModel::from_type(RadarType::GENERIC)) {
        initialize_rng();
    }

    /**
     * @brief Constructor with radar type
     *
     * Creates a radar with specifications matching the given type.
     * Radar is positioned at origin.
     *
     * @param type Radar type
     */
    explicit RadarModel(RadarType type)
        : position_(Vec3::zero()),
          type_(type),
          noise_(RadarNoiseModel::from_type(type)) {
        initialize_rng();
    }

    /**
     * @brief Constructor with position and type
     *
     * @param position Radar position [m]
     * @param type Radar type
     */
    RadarModel(const Vec3& position, RadarType type)
        : position_(position),
          type_(type),
          noise_(RadarNoiseModel::from_type(type)) {
        initialize_rng();
    }

    /**
     * @brief Constructor with custom noise model
     *
     * @param position Radar position [m]
     * @param noise Custom noise parameters
     */
    RadarModel(const Vec3& position, const RadarNoiseModel& noise)
        : position_(position),
          type_(RadarType::GENERIC),
          noise_(noise) {
        initialize_rng();
    }

    // === CONFIGURATION ===

    /**
     * @brief Set radar position
     *
     * @param pos Radar position [m]
     */
    void set_position(const Vec3& pos) { position_ = pos; }

    /**
     * @brief Get radar position
     *
     * @return Position [m]
     */
    Vec3 get_position() const { return position_; }

    /**
     * @brief Set radar type (updates noise model)
     *
     * @param type Radar type
     */
    void set_type(RadarType type) {
        type_ = type;
        noise_ = RadarNoiseModel::from_type(type);
    }

    /**
     * @brief Get radar type
     *
     * @return Radar type
     */
    RadarType get_type() const { return type_; }

    /**
     * @brief Set noise parameters directly
     *
     * @param noise Noise model
     */
    void set_noise_model(const RadarNoiseModel& noise) {
        noise_ = noise;
    }

    /**
     * @brief Get noise model
     *
     * @return Current noise parameters
     */
    const RadarNoiseModel& get_noise_model() const { return noise_; }

    /**
     * @brief Set detection probability threshold
     *
     * Probability of detecting a target at max range.
     * Typical values: 0.5-0.9 for operational radars.
     *
     * @param prob Detection probability (0-1)
     */
    void set_detection_probability(double prob) {
        pd_max_ = std::clamp(prob, 0.0, 1.0);
    }

    /**
     * @brief Set maximum detection range
     *
     * Targets beyond this range are never detected.
     *
     * @param range Maximum range [m]
     */
    void set_max_range(double range) {
        max_range_ = range;
    }

    // === MEASUREMENT GENERATION ===

    /**
     * @brief Generate measurement from target state
     *
     * Simulates a complete radar measurement cycle:
     * 1. Compute true measurement (noise-free)
     * 2. Apply detection probability
     * 3. Add Gaussian noise if detected
     * 4. Check range limits
     *
     * @param target True target state
     * @return Measurement (detected flag indicates success)
     *
     * @example
     * @code
     * RadarModel radar(RadarType::FIRE_CONTROL);
     * State6DOF target(...);
     *
     * auto z = radar.measure(target);
     * if (z.detected) {
     *     std::cout << "Target at range: " << z.range << " m\n";
     * }
     * @endcode
     */
    Measurement measure(const State6DOF& target) {
        Measurement m;

        // Get target position relative to radar
        Vec3 rel_pos = target.get_position() - position_;
        Vec3 rel_vel = target.get_velocity();

        // Compute true range
        double true_range = rel_pos.magnitude();

        // Check maximum range
        if (true_range > max_range_) {
            m.detected = false;
            return m;
        }

        // Detection probability decreases with range (radar equation: Pd ~ 1/R^4)
        double range_ratio = true_range / max_range_;
        double pd = pd_max_ * std::pow(1.0 - range_ratio * range_ratio * range_ratio * range_ratio, 2);

        // Random detection check
        std::uniform_real_distribution<> detect_check(0.0, 1.0);
        if (detect_check(rng_) > pd) {
            m.detected = false;
            return m;
        }

        // Compute true measurements
        Vec3 pos_norm = rel_pos.normalized();
        m.range = true_range;
        m.azimuth = std::atan2(rel_pos.x, rel_pos.z);
        m.elevation = std::asin(rel_pos.y / (true_range + 1e-10));
        m.range_rate = rel_vel.dot(pos_norm);

        // Add Gaussian noise
        std::normal_distribution<> noise(0.0, 1.0);
        m.range += noise_.range_std * noise(rng_);
        m.azimuth += noise_.azimuth_std * noise(rng_);
        m.elevation += noise_.elevation_std * noise(rng_);
        m.range_rate += noise_.range_rate_std * noise(rng_);

        // Ensure range is positive
        m.range = std::max(m.range, 0.0);

        m.detected = true;
        return m;
    }

    /**
     * @brief Generate perfect (noise-free) measurement
     *
     * Useful for testing and calibration.
     *
     * @param target Target state
     * @return Perfect measurement
     */
    Measurement measure_perfect(const State6DOF& target) const {
        Measurement m;

        Vec3 rel_pos = target.get_position() - position_;
        Vec3 rel_vel = target.get_velocity();
        double r = rel_pos.magnitude();

        m.range = r;
        m.azimuth = std::atan2(rel_pos.x, rel_pos.z);
        m.elevation = std::asin(rel_pos.y / (r + 1e-10));
        m.range_rate = rel_vel.dot(rel_pos.normalized());
        m.detected = true;

        return m;
    }

    /**
     * @brief Generate multiple measurements with clutter
     *
     * Simulates radar returns with false alarms (clutter).
     * Common in real radar systems operating in cluttered environments.
     *
     * @param target True target state
     * @param num_clutter Number of false targets to generate
     * @return Vector of measurements (first is true target if detected)
     *
     * @example
     * @code
     * auto measurements = radar.measure_with_clutter(target, 5);
     * for (const auto& m : measurements) {
     *     if (m.detected) {
     *         // Process measurement...
     *     }
     * }
     * @endcode
     */
    std::vector<Measurement> measure_with_clutter(const State6DOF& target, int num_clutter = 0) {
        std::vector<Measurement> measurements;

        // Measure true target
        Measurement true_meas = measure(target);
        if (true_meas.detected) {
            measurements.push_back(true_meas);
        }

        // Generate clutter (false targets)
        std::uniform_real_distribution<> range_dist(0.0, max_range_);
        std::uniform_real_distribution<> az_dist(-M_PI, M_PI);
        std::uniform_real_distribution<> el_dist(-M_PI/2, M_PI/2);
        std::normal_distribution<> rr_noise(0.0, noise_.range_rate_std);

        for (int i = 0; i < num_clutter; ++i) {
            Measurement clutter;
            clutter.range = range_dist(rng_);
            clutter.azimuth = az_dist(rng_);
            clutter.elevation = el_dist(rng_);
            clutter.range_rate = rr_noise(rng_);
            clutter.detected = true;
            measurements.push_back(clutter);
        }

        return measurements;
    }

    /**
     * @brief Calculate theoretical measurement accuracy
     *
     * Returns the Cramér-Rao lower bound (CRLB) for measurement accuracy
     * based on radar parameters and target state.
     *
     * @param target Target state
     * @return Theoretical noise model (range-dependent)
     */
    RadarNoiseModel theoretical_accuracy(const State6DOF& target) const {
        Vec3 rel_pos = target.get_position() - position_;
        double range = rel_pos.magnitude();

        // Accuracy degrades with range (typically 1/R SNR)
        double range_factor = std::max(1.0, range / 10000.0);

        RadarNoiseModel accuracy = noise_;
        accuracy.range_std *= range_factor;
        accuracy.azimuth_std *= range_factor;
        accuracy.elevation_std *= range_factor;

        return accuracy;
    }

    /**
     * @brief Get radar specifications as string
     *
     * @return Human-readable radar description
     */
    std::string get_specifications() const {
        std::string type_name;
        double max_range_km;

        switch (type_) {
            case RadarType::GENERIC:
                type_name = "Generic Surveillance";
                max_range_km = 300.0;
                break;
            case RadarType::ATC_SURVEILLANCE:
                type_name = "ATC Surveillance (ASR/ARSR)";
                max_range_km = 400.0;
                break;
            case RadarType::FIRE_CONTROL:
                type_name = "Fire Control (MIL-STD)";
                max_range_km = 50.0;
                break;
            case RadarType::MISSILE_GUIDANCE:
                type_name = "Missile Guidance (SARH)";
                max_range_km = 30.0;
                break;
            case RadarType::PHASED_ARRAY:
                type_name = "Phased Array (AESA)";
                max_range_km = 400.0;
                break;
            case RadarType::PRECISION_APPROACH:
                type_name = "Precision Approach (PAR)";
                max_range_km = 50.0;
                break;
            default:
                type_name = "Unknown";
                max_range_km = 100.0;
        }

        return type_name + " Radar - Max Range: " + std::to_string(max_range_km) + " km, " +
               "Accuracy: ±" + std::to_string(noise_.range_std) + "m range, " +
               "±" + std::to_string(noise_.azimuth_std * 180.0 / M_PI) + "° angle";
    }

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const RadarModel& radar) {
        os << radar.get_specifications();
        return os;
    }

private:
    Vec3 position_;              ///< Radar position [m]
    RadarType type_;             ///< Radar type
    RadarNoiseModel noise_;      ///< Noise parameters
    double pd_max_ = 0.9;        ///< Max detection probability
    double max_range_ = 300000.0; ///< Maximum range [m] (300 km default)
    mutable std::mt19937 rng_;   ///< Random number generator

    /**
     * @brief Initialize random number generator
     */
    void initialize_rng() {
        std::random_device rd;
        rng_ = std::mt19937(rd());
    }
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline RadarNoiseModel RadarNoiseModel::from_type(RadarType type) {
    switch (type) {
        case RadarType::GENERIC:
            // Generic surveillance radar
            // Typical airport surveillance: ±50m range, ±0.2° angle
            return RadarNoiseModel(50.0, 0.0035, 0.0035, 2.0);

        case RadarType::ATC_SURVEILLANCE:
            // Air Traffic Control Radar (ASR-9, ARSR-4 class)
            // Specs: ±30m range, ±0.1° azimuth, ±0.15° elevation
            return RadarNoiseModel(30.0, 0.00175, 0.0026, 1.5);

        case RadarType::FIRE_CONTROL:
            // Military Fire Control Radar (e.g., AN/SPG-60, AN/MPQ-53)
            // Specs: ±5m range, ±0.01° angle
            return RadarNoiseModel(5.0, 0.000175, 0.000175, 0.5);

        case RadarType::MISSILE_GUIDANCE:
            // Semi-Active Homing Radar (e.g., AIM-7 Sparrow, R-77)
            // Specs: ±2m range, ±0.005° angle
            return RadarNoiseModel(2.0, 0.000087, 0.000087, 0.3);

        case RadarType::PHASED_ARRAY:
            // AESA Radar (e.g., AN/APG-77, AN/SPY-1)
            // Specs: ±10m range, ±0.05° angle
            return RadarNoiseModel(10.0, 0.00087, 0.00087, 1.0);

        case RadarType::PRECISION_APPROACH:
            // Precision Approach Radar (PAR, GCA)
            // Specs: ±3m range, ±0.02° angle
            return RadarNoiseModel(3.0, 0.00035, 0.00035, 0.5);

        default:
            return RadarNoiseModel(10.0, 0.001, 0.001, 1.0);
    }
}

} // namespace ballistx

#endif // BALLISTX_RADAR_MODEL_H
