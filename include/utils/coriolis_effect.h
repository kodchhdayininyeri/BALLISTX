#ifndef BALLISTX_CORIOLIS_EFFECT_H
#define BALLISTX_CORIOLIS_EFFECT_H

#include "utils/vec3.h"
#include <cmath>

namespace ballistx {

/**
 * @brief Coriolis Effect for Long-Range Ballistics
 *
 * Due to Earth's rotation, moving objects experience an apparent deflection
 * called the Coriolis effect. This is critical for long-range artillery,
 * ICBMs, and precision sniping.
 *
 * Formula: F_coriolis = -2 × m × (Ω × v)
 *
 * Where:
 *   m = mass of projectile (kg)
 *   Ω = Earth's angular velocity vector (rad/s)
 *   v = velocity vector (m/s)
 *
 * Physical explanation:
 * - Earth rotates eastward at angular velocity Ω
 * - Object moving northward has higher eastward velocity at equator
 * - Appears to deflect to RIGHT in northern hemisphere
 * - Appears to deflect to LEFT in southern hemisphere
 *
 * Applications:
 * - Long-range artillery (20km+)
 * - Sniper shots (1000m+)
 * - ICBM trajectory calculations
 * - Weather pattern prediction
 */
class CoriolisEffect {
public:
    // Earth's rotation parameters
    static constexpr double EARTH_OMEGA_MAGNITUDE = 7.2921e-5;  // rad/s
    static constexpr double EARTH_ROTATION_PERIOD = 86164.09;  // seconds (sidereal day)
    static constexpr double EARTH_RADIUS = 6371000.0;        // meters

    /**
     * @brief Calculate Coriolis acceleration
     *
     * @param velocity Object velocity (m/s)
     * @param latitude Latitude in radians (positive for north)
     * @return Coriolis acceleration (m/s²)
     */
    static Vec3 calculate_acceleration(const Vec3& velocity, double latitude) {
        // Earth's angular velocity vector (pointing north along Earth's axis)
        Vec3 omega = get_earth_omega_vector(latitude);

        // Coriolis acceleration: a_c = -2 × (Ω × v)
        Vec3 omega_cross_v = omega.cross(velocity);
        Vec3 coriolis_accel = omega_cross_v * (-2.0);

        return coriolis_accel;
    }

    /**
     * @brief Calculate Coriolis force
     *
     * @param velocity Object velocity (m/s)
     * @param latitude Latitude in radians
     * @param mass Object mass (kg)
     * @return Coriolis force (Newtons)
     */
    static Vec3 calculate_force(const Vec3& velocity, double latitude, double mass) {
        Vec3 accel = calculate_acceleration(velocity, latitude);
        return accel * mass;
    }

    /**
     * @brief Calculate Coriolis acceleration with full Earth rotation vector
     *
     * More accurate version that accounts for Earth's 3D rotation
     *
     * @param velocity Object velocity (m/s)
     * @param latitude Latitude in radians
     * @param longitude Longitude in radians (optional, for global calculations)
     * @return Coriolis acceleration (m/s²)
     */
    static Vec3 calculate_acceleration_3d(const Vec3& velocity,
                                            double latitude,
                                            double longitude = 0.0) {
        // Full 3D Earth angular velocity vector
        Vec3 omega = get_earth_omega_vector_3d(latitude, longitude);

        // a_c = -2 × (Ω × v)
        Vec3 omega_cross_v = omega.cross(velocity);
        return omega_cross_v * (-2.0);
    }

    /**
     * @brief Estimate lateral deflection due to Coriolis effect
     *
     * Simplified formula for east-west and north-south deflection
     *
     * @param range Target range (m)
     * @param latitude Latitude in radians
     * @param flight_time Total flight time (s)
     * @param azimuth Firing azimuth (radians, 0 = north, 90° = east)
     * @return Lateral deflection (meters)
     */
    static Vec3 estimate_lateral_deflection(double range,
                                           double latitude,
                                           double flight_time,
                                           double azimuth) {
        // Simplified Coriolis deflection formulas
        double omega = EARTH_OMEGA_MAGNITUDE * std::sin(latitude);
        double deflection_north = 2.0 * omega * range * flight_time * std::cos(azimuth);
        double deflection_east = 2.0 * omega * range * flight_time * std::sin(azimuth);

        return Vec3(deflection_east, 0.0, deflection_north);
    }

    /**
     * @brief Calculate deflection for artillery shell
     *
     * @param range Target range (m)
     * @param flight_time Flight time (s)
     * @param latitude Latitude in degrees
     * @param azimuth Azimuth in degrees
     * @return [east_deflection, north_deflection] in meters
     */
    static std::pair<double, double> calculate_artillery_deflection(
        double range,
        double flight_time,
        double latitude_degrees,
        double azimuth_degrees) {

        double latitude = latitude_degrees * M_PI / 180.0;
        double azimuth = azimuth_degrees * M_PI / 180.0;
        double omega = EARTH_OMEGA_MAGNITUDE;

        // Horizontal component of Earth's rotation
        double omega_horizontal = omega * std::sin(latitude);

        // Deflection components
        double deflection_east = omega_horizontal * range * flight_time * std::sin(azimuth);
        double deflection_north = omega_horizontal * range * flight_time * std::cos(azimuth);

        return {deflection_east, deflection_north};
    }

    /**
     * @brief Calculate correction for sniper shot
     *
     * @param range Target range (m)
     * @param latitude Latitude in degrees
     * @param azimuth Azimuth to target (degrees, 0 = north)
     * @return Correction in MOA (Minutes of Angle) and inches
     */
    static std::pair<double, double> calculate_sniper_correction(
        double range,
        double latitude_degrees,
        double azimuth_degrees) {

        // Estimate flight time for typical sniper bullet
        double velocity = 800.0;  // m/s
        double flight_time = range / velocity;

        auto [deflection_east, deflection_north] = calculate_artillery_deflection(
            range, flight_time, latitude_degrees, azimuth_degrees
        );

        // Total lateral deflection
        double total_deflection = std::sqrt(deflection_east * deflection_east +
                                              deflection_north * deflection_north);

        // Convert to MOA (1 MOA = 1/60 degree at 100 yards)
        double deflection_moa = (total_deflection / range) * (180.0 / M_PI) * 60.0;

        // Convert to inches at 100 yards
        double deflection_inches = total_deflection * 0.0393701;  // m to inches

        return {deflection_moa, deflection_inches};
    }

    /**
     * @brief Get Earth's angular velocity vector at latitude
     *
     * @param latitude Latitude in radians
     * @return Angular velocity vector (rad/s)
     */
    static Vec3 get_earth_omega_vector(double latitude) {
        // Earth rotates around polar axis (Z axis in local coordinates)
        double omega_z = EARTH_OMEGA_MAGNITUDE * std::sin(latitude);
        double omega_y = EARTH_OMEGA_MAGNITUDE * std::cos(latitude);

        return Vec3(0.0, omega_y, omega_z);
    }

    /**
     * @brief Get full 3D Earth angular velocity vector
     *
     * @param latitude Latitude in radians
     * @param longitude Longitude in radians
     * @return Angular velocity in ECEF coordinates (rad/s)
     */
    static Vec3 get_earth_omega_vector_3d(double latitude, double longitude) {
        double omega = EARTH_OMEGA_MAGNITUDE;

        // Earth rotation axis through poles
        double omega_x = 0.0;
        double omega_y = omega * std::cos(latitude);
        double omega_z = omega * std::sin(latitude);

        // Rotate by longitude (simplified)
        Vec3 omega_local(omega_x, omega_y, omega_z);

        // For most ballistic calculations, we can ignore longitude effects
        // on the magnitude of omega, only the direction matters
        return omega_local;
    }

    /**
     * @brief Calculate Coriolis frequency (inertial oscillation frequency)
     *
     * For atmospheric and ocean dynamics
     *
     * @param latitude Latitude in radians
     * @return Coriolis frequency (rad/s)
     */
    static double get_coriolis_frequency(double latitude) {
        return 2.0 * EARTH_OMEGA_MAGNITUDE * std::sin(latitude);
    }

    /**
     * @brief Get Coriolis parameter for calculations
     *
     * f = 2Ω × sin(φ)
     * Used in meteorology and oceanography
     *
     * @param latitude Latitude in radians
     * @return Coriolis parameter (rad/s)
     */
    static double get_coriolis_parameter(double latitude) {
        return get_coriolis_frequency(latitude);
    }

    /**
     * @brief Check if Coriolis effect is significant for given parameters
     *
     * @param range Target range (m)
     * @param flight_time Flight time (s)
     * @param latitude Latitude in degrees
     * @return True if deflection > 0.1m
     */
    static bool is_significant(double range,
                             double flight_time,
                             double latitude_degrees) {
        auto [deflection_east, deflection_north] = calculate_artillery_deflection(
            range, flight_time, latitude_degrees, 45.0
        );

        double total_deflection = std::sqrt(deflection_east * deflection_east +
                                              deflection_north * deflection_north);

        return total_deflection > 0.1;  // 10 cm threshold
    }

    /**
     * @brief Calculate latitude-dependent deflection multiplier
     *
     * Coriolis effect is zero at equator, maximum at poles
     *
     * @param latitude_degrees Latitude in degrees
     * @return Multiplier (0.0 at equator, 1.0 at poles)
     */
    static double get_latitude_multiplier(double latitude_degrees) {
        return std::abs(std::sin(latitude_degrees * M_PI / 180.0));
    }

    /**
     * @brief Calculate deflection for specific scenario
     *
     * @param velocity Launch velocity (m/s)
     * @param launch_angle Launch angle (radians)
     * @param latitude_degrees Latitude in degrees
     * @param azimuth_degrees Azimuth in degrees
     * @return [range, deflection_right, deflection_up] in meters
     */
    static std::tuple<double, double, double> calculate_trajectory_deflection(
        double velocity,
        double launch_angle,
        double latitude_degrees,
        double azimuth_degrees) {

        // Estimate range and flight time (simplified vacuum trajectory)
        double g = 9.80665;
        double v_x = velocity * std::cos(launch_angle);
        double v_y = velocity * std::sin(launch_angle);
        double flight_time = 2.0 * v_y / g;
        double range = v_x * flight_time;

        // Calculate Coriolis deflection
        auto [deflection_east, deflection_north] = calculate_artillery_deflection(
            range, flight_time, latitude_degrees, azimuth_degrees
        );

        // Rotate deflection to firing coordinates
        double azimuth_rad = azimuth_degrees * M_PI / 180.0;
        double deflection_right = deflection_east * std::cos(azimuth_rad) +
                                   deflection_north * std::sin(azimuth_rad);
        double deflection_up = 0.0;  // Negligible for horizontal artillery

        return {range, deflection_right, deflection_up};
    }

    /**
     * @brief Explain Coriolis effect direction
     *
     * @param latitude_degrees Latitude in degrees
     * @return Direction description
     */
    static std::string get_deflection_direction(double latitude_degrees) {
        if (latitude_degrees > 0.0) {
            return "RIGHT (Northern Hemisphere)";
        } else if (latitude_degrees < 0.0) {
            return "LEFT (Southern Hemisphere)";
        } else {
            return "NONE (Equator - no Coriolis effect)";
        }
    }

    /**
     * @brief Print Coriolis effect summary for location
     *
     * @param latitude_degrees Latitude in degrees
     * @param longitude_degrees Longitude in degrees
     */
    static void print_location_info(double latitude_degrees,
                                   double longitude_degrees = 0.0) {
        std::cout << "=== Coriolis Effect at " << latitude_degrees << "° latitude ===\n";
        std::cout << "Deflection Direction: " << get_deflection_direction(latitude_degrees) << "\n";
        std::cout << "Latitude Multiplier: " << (get_latitude_multiplier(latitude_degrees) * 100.0)
                  << "%\n";
        std::cout << "Coriolis Parameter: " << (get_coriolis_parameter(latitude_degrees * M_PI / 180.0) * 1e5)
                  << "e-5 rad/s\n";
        std::cout << "Significant for ranges > " << (100.0 / get_latitude_multiplier(latitude_degrees))
                  << " m\n";
    }
};

} // namespace ballistx

#endif // BALLISTX_CORIOLIS_EFFECT_H
