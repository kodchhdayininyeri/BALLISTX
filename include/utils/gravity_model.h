#ifndef BALLISTX_GRAVITY_MODEL_H
#define BALLISTX_GRAVITY_MODEL_H

#include "utils/vec3.h"
#include <cmath>

namespace ballistx {

/**
 * @brief Gravity model with altitude dependence
 *
 * Standard gravity: g = 9.80665 m/s² at sea level
 * Altitude-dependent: g(h) = g₀ × (R / (R + h))²
 *
 * Where:
 *   g₀ = 9.80665 m/s² (standard gravity at sea level)
 *   R = 6,371,000 m (Earth's mean radius)
 *   h = altitude (meters above sea level)
 *
 * This accounts for the inverse-square law of gravitation.
 * For artillery altitudes (0-50km), the difference is small
 * but becomes significant for long-range projectiles and rockets.
 */
class GravityModel {
public:
    // Earth parameters
    static constexpr double EARTH_RADIUS = 6371000.0;      // meters
    static constexpr double STANDARD_GRAVITY = 9.80665;    // m/s² at sea level
    static constexpr double GRAVITATIONAL_CONSTANT = 6.674e-11;  // m³/(kg·s²)
    static constexpr double EARTH_MASS = 5.972e24;         // kg

    // Constructors
    GravityModel() = default;

    /**
     * @brief Calculate gravity acceleration at altitude
     *
     * @param altitude Altitude above sea level (meters)
     * @return Gravity acceleration (m/s²)
     */
    static double get_gravity(double altitude = 0.0) {
        // g(h) = g₀ × (R / (R + h))²
        double factor = EARTH_RADIUS / (EARTH_RADIUS + altitude);
        return STANDARD_GRAVITY * factor * factor;
    }

    /**
     * @brief Calculate gravity vector at position
     *
     * @param position 3D position (x, y, z) in meters
     * @return Gravity vector (always points down in Y direction)
     */
    static Vec3 get_gravity_vector(const Vec3& position) {
        return Vec3(0.0, -get_gravity(position.y), 0.0);
    }

    /**
     * @brief Get gravitational force on a mass
     *
     * @param mass Mass in kg
     * @param altitude Altitude in meters
     * @return Force vector in Newtons
     */
    static Vec3 get_gravitational_force(double mass, double altitude = 0.0) {
        return Vec3(0.0, -mass * get_gravity(altitude), 0.0);
    }

    /**
     * @brief Calculate gravity using Newton's law of universal gravitation
     *
     * More accurate for very high altitudes:
     * g = G × M / (R + h)²
     *
     * @param altitude Altitude above sea level (meters)
     * @return Gravity acceleration (m/s²)
     */
    static double get_gravity_newton(double altitude = 0.0) {
        double r = EARTH_RADIUS + altitude;
        return (GRAVITATIONAL_CONSTANT * EARTH_MASS) / (r * r);
    }

    /**
     * @brief Compare standard vs altitude-dependent gravity
     *
     * @param altitude Altitude in meters
     * @return Difference in gravity (m/s²)
     */
    static double get_gravity_difference(double altitude) {
        return STANDARD_GRAVITY - get_gravity(altitude);
    }

    /**
     * @brief Calculate gravity at multiple altitudes for analysis
     */
    struct GravityProfile {
        double altitude;
        double gravity_standard;
        double gravity_altitude;
        double difference_percent;
    };

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

    // Quick reference values
    static constexpr double get_standard_gravity() { return STANDARD_GRAVITY; }
    static constexpr double get_earth_radius() { return EARTH_RADIUS; }
};

} // namespace ballistx

#endif // BALLISTX_GRAVITY_MODEL_H
