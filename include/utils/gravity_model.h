#ifndef BALLISTX_GRAVITY_MODEL_H
#define BALLISTX_GRAVITY_MODEL_H

#include "utils/vec3.h"
#include <cmath>
#include <vector>

namespace ballistx {

/**
 * @brief Gravity model with altitude dependence
 *
 * Calculates gravitational acceleration for ballistics simulations.
 * Accounts for the inverse-square law of gravitation with altitude.
 *
 * **Physics Model:**
 * - Standard gravity at sea level: g₀ = 9.80665 m/s²
 * - Altitude-dependent formula: g(h) = g₀ × (R / (R + h))²
 * - Newton's universal gravitation: g = G × M / (R + h)²
 *
 * **Where:**
 * - g₀ = 9.80665 m/s² (standard gravity at sea level)
 * - R = 6,371,000 m (Earth's mean radius)
 * - h = altitude (meters above sea level)
 * - G = 6.674×10⁻¹¹ m³/(kg·s²) (gravitational constant)
 * - M = 5.972×10²⁴ kg (Earth's mass)
 *
 * **Accuracy:**
 * - For artillery altitudes (0-50 km): difference is < 2%
 * - For ballistic missiles (> 100 km): significant impact
 * - For orbital trajectories: essential for accuracy
 *
 * @example
 * @code
 * // Get gravity at sea level
 * double g0 = GravityModel::get_gravity();  // 9.80665 m/s²
 *
 * // Get gravity at 10 km altitude
 * double g10k = GravityModel::get_gravity(10000.0);  // ~9.79 m/s²
 *
 * // Get gravity vector at position
 * Vec3 position(0.0, 5000.0, 0.0);  // 5 km up
 * Vec3 gravity = GravityModel::get_gravity_vector(position);
 *
 * // Calculate force on 50 kg projectile
 * Vec3 force = GravityModel::get_gravitational_force(50.0, 5000.0);
 *
 * // Analyze gravity profile up to 50 km
 * auto profile = GravityModel::analyze_altitude_range(50000.0);
 * @endcode
 *
 * @see CoriolisEffect for Earth rotation effects
 * @see Atmosphere for air density models
 */
class GravityModel {
public:
    /**
     * @brief Earth's mean radius
     *
     * Average distance from Earth's center to surface: 6,371 km
     */
    static constexpr double EARTH_RADIUS = 6371000.0;  ///< meters

    /**
     * @brief Standard gravity at sea level
     *
     * Defined by International Bureau of Weights and Measures (1960)
     */
    static constexpr double STANDARD_GRAVITY = 9.80665;  ///< m/s² at sea level

    /**
     * @brief Universal gravitational constant
     *
     * G = 6.674×10⁻¹¹ m³/(kg·s²)
     */
    static constexpr double GRAVITATIONAL_CONSTANT = 6.674e-11;  ///< m³/(kg·s²)

    /**
     * @brief Earth's mass
     *
     * M = 5.972×10²⁴ kg
     */
    static constexpr double EARTH_MASS = 5.972e24;  ///< kg

    /**
     * @brief Default constructor
     */
    GravityModel() = default;

    /**
     * @brief Calculate gravity acceleration at altitude
     *
     * Uses the inverse-square law approximation:
     * g(h) = g₀ × (R / (R + h))²
     *
     * This formula derives from Newton's law of universal gravitation
     * and provides excellent accuracy for altitudes up to ~100 km.
     *
     * @param altitude Altitude above sea level in meters (default: 0.0)
     * @return Gravity acceleration in m/s²
     *
     * @example
     * @code
     * double g_sea_level = GravityModel::get_gravity(0.0);    // 9.80665
     * double g_10km = GravityModel::get_gravity(10000.0);     // 9.7904
     * double g_50km = GravityModel::get_gravity(50000.0);     // 9.6582
     * @endcode
     */
    static double get_gravity(double altitude = 0.0) {
        // g(h) = g₀ × (R / (R + h))²
        double factor = EARTH_RADIUS / (EARTH_RADIUS + altitude);
        return STANDARD_GRAVITY * factor * factor;
    }

    /**
     * @brief Calculate gravity vector at position
     *
     * Returns a 3D gravity vector pointing downward (negative Y direction).
     * The magnitude varies with altitude based on the Y component.
     *
     * @param position 3D position (x, y, z) in meters
     * @return Gravity vector pointing down: (0, -g, 0)
     *
     * @example
     * @code
     * Vec3 pos(1000.0, 5000.0, 2000.0);  // 5 km altitude
     * Vec3 g = GravityModel::get_gravity_vector(pos);
     * // g = (0, -9.79, 0) m/s²
     * @endcode
     */
    static Vec3 get_gravity_vector(const Vec3& position) {
        return Vec3(0.0, -get_gravity(position.y), 0.0);
    }

    /**
     * @brief Get gravitational force on a mass
     *
     * Calculates the gravitational force: F = m × g
     * Returns a 3D force vector in Newtons.
     *
     * @param mass Mass in kilograms
     * @param altitude Altitude in meters (default: 0.0)
     * @return Force vector in Newtons: (0, -m×g, 0)
     *
     * @example
     * @code
     * // 50 kg projectile at sea level
     * Vec3 force = GravityModel::get_gravitational_force(50.0);
     * // force = (0, -490.3, 0) N
     *
     * // Same projectile at 10 km altitude
     * Vec3 force_10k = GravityModel::get_gravitational_force(50.0, 10000.0);
     * // force_10k = (0, -489.5, 0) N (slightly less)
     * @endcode
     */
    static Vec3 get_gravitational_force(double mass, double altitude = 0.0) {
        return Vec3(0.0, -mass * get_gravity(altitude), 0.0);
    }

    /**
     * @brief Calculate gravity using Newton's law of universal gravitation
     *
     * More accurate for very high altitudes and orbital calculations.
     * Uses the fundamental formula: g = G × M / (R + h)²
     *
     * This is equivalent to get_gravity() but uses the physical constants
     * directly rather than the sea-level gravity constant.
     *
     * @param altitude Altitude above sea level in meters (default: 0.0)
     * @return Gravity acceleration in m/s²
     *
     * @example
     * @code
     * double g = GravityModel::get_gravity_newton(0.0);
     * // g ≈ 9.80665 m/s² (same as standard gravity)
     *
     * double g_orbit = GravityModel::get_gravity_newton(400000.0);
     * // g ≈ 8.69 m/s² at ISS altitude (~400 km)
     * @endcode
     */
    static double get_gravity_newton(double altitude = 0.0) {
        double r = EARTH_RADIUS + altitude;
        return (GRAVITATIONAL_CONSTANT * EARTH_MASS) / (r * r);
    }

    /**
     * @brief Compare standard vs altitude-dependent gravity
     *
     * Returns the absolute difference between sea-level gravity
     * and gravity at the specified altitude.
     *
     * @param altitude Altitude in meters
     * @return Difference in gravity (m/s²)
     *
     * @example
     * @code
     * double diff_10k = GravityModel::get_gravity_difference(10000.0);
     * // diff_10k ≈ 0.016 m/s²
     * @endcode
     */
    static double get_gravity_difference(double altitude) {
        return STANDARD_GRAVITY - get_gravity(altitude);
    }

    /**
     * @brief Gravity profile data point
     *
     * Contains gravity values and statistics for a specific altitude.
     */
    struct GravityProfile {
        double altitude;              ///< Altitude above sea level [m]
        double gravity_standard;      ///< Standard gravity at sea level [m/s²]
        double gravity_altitude;      ///< Gravity at altitude [m/s²]
        double difference_percent;    ///< Percent difference from sea level
    };

    /**
     * @brief Calculate gravity at multiple altitudes for analysis
     *
     * Generates a profile of gravity values from sea level to max_altitude.
     * Useful for plotting gravity variation and analyzing trajectory effects.
     *
     * @param max_altitude Maximum altitude to analyze [m]
     * @param step Altitude increment between data points [m] (default: 1000 m)
     * @return Vector of GravityProfile data points
     *
     * @example
     * @code
     * auto profile = GravityModel::analyze_altitude_range(50000.0, 5000.0);
     * for (const auto& p : profile) {
     *     std::cout << "Alt: " << p.altitude << " m, "
     *               << "g: " << p.gravity_altitude << " m/s², "
     *               << "diff: " << p.difference_percent << "%" << std::endl;
     * }
     * @endcode
     */
    static std::vector<GravityProfile> analyze_altitude_range(
        double max_altitude,
        double step = 1000.0
    ) {
        std::vector<GravityProfile> profile;

        for (double h = 0.0; h <= max_altitude; h += step) {
            double g_alt = get_gravity(h);
            double diff_percent = ((STANDARD_GRAVITY - g_alt) / STANDARD_GRAVITY) * 100.0;

            profile.push_back({h, STANDARD_GRAVITY, g_alt, diff_percent});
        }

        return profile;
    }

    /**
     * @brief Get standard gravity constant
     *
     * @return Standard gravity at sea level: 9.80665 m/s²
     */
    static constexpr double get_standard_gravity() { return STANDARD_GRAVITY; }

    /**
     * @brief Get Earth radius constant
     *
     * @return Earth's mean radius: 6,371,000 m
     */
    static constexpr double get_earth_radius() { return EARTH_RADIUS; }
};

} // namespace ballistx

#endif // BALLISTX_GRAVITY_MODEL_H
