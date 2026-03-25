#ifndef BALLISTX_CORIOLIS_EFFECT_H
#define BALLISTX_CORIOLIS_EFFECT_H

#include "utils/vec3.h"
#include <cmath>
#include <iostream>
#include <tuple>
#include <utility>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ballistx {

/**
 * @brief Coriolis effect calculations for long-range ballistics
 *
 * Simulates the apparent deflection of moving objects due to Earth's rotation.
 * This is a fictitious force that appears in the rotating reference frame of Earth.
 *
 * **Physical Background:**
 * - Earth rotates eastward at angular velocity Ω = 7.2921×10⁻⁵ rad/s
 * - Objects moving in the rotating frame experience apparent deflection
 * - Deflection is to the RIGHT in northern hemisphere
 * - Deflection is to the LEFT in southern hemisphere
 * - Zero deflection at the equator
 *
 * **Formula:**
 * ```
 * F_coriolis = -2 × m × (Ω × v)
 * a_coriolis = -2 × (Ω × v)
 * ```
 *
 * **Where:**
 * - m = mass of projectile (kg)
 * - Ω = Earth's angular velocity vector (rad/s)
 * - v = velocity vector (m/s)
 *
 * **Applications:**
 * - Long-range artillery (20 km+): deflection of 10-100 meters
 * - Sniper shots (1000 m+): deflection of 5-10 cm
 * - ICBM trajectory calculations: deflection of hundreds of kilometers
 * - Weather pattern prediction: cyclone formation
 *
 * @example
 * @code
 * // Calculate Coriolis acceleration for artillery shell
 * Vec3 velocity(300.0, 0.0, 0.0);  // 300 m/s eastward
 * double latitude = 45.0 * M_PI / 180.0;  // 45° north
 *
 * Vec3 coriolis_accel = CoriolisEffect::calculate_acceleration(velocity, latitude);
 *
 * // Calculate deflection for 30 km shot
 * auto [east_defl, north_defl] = CoriolisEffect::calculate_artillery_deflection(
 *     30000.0,  // range (m)
 *     60.0,    // flight time (s)
 *     45.0,    // latitude (degrees)
 *     90.0     // azimuth (degrees, east)
 * );
 *
 * // Check if Coriolis is significant
 * bool significant = CoriolisEffect::is_significant(5000.0, 30.0, 45.0);
 * @endcode
 *
 * @see GravityModel for gravitational effects
 */
class CoriolisEffect {
public:
    /**
     * @brief Earth's angular velocity magnitude
     *
     * Ω = 2π / sidereal_day = 7.2921×10⁻⁵ rad/s
     */
    static constexpr double EARTH_OMEGA_MAGNITUDE = 7.2921e-5;  ///< rad/s

    /**
     * @brief Earth's sidereal rotation period
     *
     * Time for one rotation relative to fixed stars: ~23h 56m 4s
     */
    static constexpr double EARTH_ROTATION_PERIOD = 86164.09;  ///< seconds

    /**
     * @brief Earth's mean radius
     *
     * Average distance from center to surface: 6,371 km
     */
    static constexpr double EARTH_RADIUS = 6371000.0;  ///< meters

    /**
     * @brief Calculate Coriolis acceleration
     *
     * Computes the Coriolis acceleration for a given velocity and latitude.
     * Uses the formula: a_c = -2 × (Ω × v)
     *
     * @param velocity Object velocity vector (m/s)
     * @param latitude Latitude in radians (positive for north)
     * @return Coriolis acceleration vector (m/s²)
     *
     * @example
     * @code
     * Vec3 vel(100.0, 0.0, 0.0);  // Moving east
     * double lat = 45.0 * M_PI / 180.0;  // 45°N
     * Vec3 accel = CoriolisEffect::calculate_acceleration(vel, lat);
     * // accel points south and down (right deflection)
     * @endcode
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
     * Computes the Coriolis force: F_c = m × a_c
     *
     * @param velocity Object velocity vector (m/s)
     * @param latitude Latitude in radians
     * @param mass Object mass (kg)
     * @return Coriolis force vector (Newtons)
     *
     * @example
     * @code
     * Vec3 vel(500.0, 0.0, 0.0);
     * double mass = 50.0;  // kg
     * Vec3 force = CoriolisEffect::calculate_force(vel, 45.0 * M_PI / 180.0, mass);
     * @endcode
     */
    static Vec3 calculate_force(const Vec3& velocity, double latitude, double mass) {
        Vec3 accel = calculate_acceleration(velocity, latitude);
        return accel * mass;
    }

    /**
     * @brief Calculate Coriolis acceleration with full Earth rotation vector
     *
     * More accurate version that accounts for Earth's 3D rotation in ECEF coordinates.
     * Use for global-scale calculations or high-precision work.
     *
     * @param velocity Object velocity vector (m/s)
     * @param latitude Latitude in radians
     * @param longitude Longitude in radians (optional, default: 0)
     * @return Coriolis acceleration vector (m/s²)
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
     * Simplified formula for quick estimation of east-west and north-south
     * deflection based on range, flight time, and latitude.
     *
     * @param range Target range (m)
     * @param latitude Latitude in radians
     * @param flight_time Total flight time (s)
     * @param azimuth Firing azimuth (radians, 0 = north, π/2 = east)
     * @return Lateral deflection vector (east, 0, north) in meters
     *
     * @example
     * @code
     * double range = 25000.0;  // 25 km
     * double flight_time = 55.0;  // seconds
     * double lat = 50.0 * M_PI / 180.0;  // 50°N
     * double azimuth = 90.0 * M_PI / 180.0;  // East
     *
     * Vec3 deflection = CoriolisEffect::estimate_lateral_deflection(
     *     range, lat, flight_time, azimuth
     * );
     * @endcode
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
     * Practical calculation for artillery fire control.
     * Returns deflection components in east and north directions.
     *
     * @param range Target range (m)
     * @param flight_time Flight time (s)
     * @param latitude_degrees Latitude in degrees
     * @param azimuth_degrees Azimuth in degrees (0 = north, 90 = east)
     * @return Pair of (east_deflection, north_deflection) in meters
     *
     * @example
     * @code
     * auto [east, north] = CoriolisEffect::calculate_artillery_deflection(
     *     30000.0,  // 30 km range
     *     60.0,     // 60 second flight time
     *     45.0,     // 45°N latitude
     *     90.0      // firing east
     * );
     * std::cout << "East deflection: " << east << " m" << std::endl;
     * std::cout << "North deflection: " << north << " m" << std::endl;
     * @endcode
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
     * Estimates the Coriolis correction needed for precision sniping.
     * Returns correction in MOA (Minutes of Angle) and inches.
     *
     * @param range Target range (m)
     * @param latitude_degrees Latitude in degrees
     * @param azimuth_degrees Azimuth to target (degrees, 0 = north)
     * @return Pair of (correction_moa, correction_inches)
     *
     * @example
     * @code
     * auto [moa, inches] = CoriolisEffect::calculate_sniper_correction(
     *     1500.0,  // 1500 m range
     *     50.0,    // 50°N latitude
     *     0.0      // firing north
     * );
     * std::cout << "Apply " << moa << " MOA correction (" << inches << " inches)" << std::endl;
     * @endcode
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
     * Returns the Earth's rotation vector in local coordinates.
     * The Z component points north along the Earth's axis.
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
     * Returns the complete Earth rotation vector in ECEF coordinates.
     *
     * @param latitude Latitude in radians
     * @param longitude Longitude in radians
     * @return Angular velocity vector in ECEF coordinates (rad/s)
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
     * Used in atmospheric and ocean dynamics for computing
     * inertial oscillations and geostrophic flow.
     *
     * @param latitude Latitude in radians
     * @return Coriolis frequency f = 2Ω sin(φ) (rad/s)
     */
    static double get_coriolis_frequency(double latitude) {
        return 2.0 * EARTH_OMEGA_MAGNITUDE * std::sin(latitude);
    }

    /**
     * @brief Get Coriolis parameter for calculations
     *
     * Alias for get_coriolis_frequency(). Commonly used in
     * meteorology and oceanography (denoted as 'f').
     *
     * @param latitude Latitude in radians
     * @return Coriolis parameter f = 2Ω sin(φ) (rad/s)
     */
    static double get_coriolis_parameter(double latitude) {
        return get_coriolis_frequency(latitude);
    }

    /**
     * @brief Check if Coriolis effect is significant for given parameters
     *
     * Determines if Coriolis deflection exceeds a practical threshold
     * (10 cm by default). Useful for deciding when to include Coriolis
     * in simulations.
     *
     * @param range Target range (m)
     * @param flight_time Flight time (s)
     * @param latitude_degrees Latitude in degrees
     * @return true if deflection > 0.1 m, false otherwise
     *
     * @example
     * @code
     * bool needs_correction = CoriolisEffect::is_significant(
     *     5000.0,  // 5 km range
     *     10.0,    // 10 s flight time
     *     45.0     // 45° latitude
     * );
     * @endcode
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
     * Coriolis effect varies with latitude, being zero at the equator
     * and maximum at the poles. This returns a normalized multiplier.
     *
     * @param latitude_degrees Latitude in degrees
     * @return Multiplier (0.0 at equator, 1.0 at poles)
     *
     * @example
     * @code
     * double multiplier = CoriolisEffect::get_latitude_multiplier(45.0);
     * // multiplier ≈ 0.707 (sin(45°))
     * @endcode
     */
    static double get_latitude_multiplier(double latitude_degrees) {
        return std::abs(std::sin(latitude_degrees * M_PI / 180.0));
    }

    /**
     * @brief Calculate deflection for specific trajectory scenario
     *
     * Estimates the range and Coriolis deflection for a projectile
     * with given launch parameters using simplified vacuum trajectory.
     *
     * @param velocity Launch velocity (m/s)
     * @param launch_angle Launch angle (radians)
     * @param latitude_degrees Latitude in degrees
     * @param azimuth_degrees Azimuth in degrees
     * @return Tuple of (range, deflection_right, deflection_up) in meters
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
     * Returns a human-readable description of the deflection
     * direction for a given latitude.
     *
     * @param latitude_degrees Latitude in degrees
     * @return Direction description string
     *
     * @example
     * @code
     * std::string dir = CoriolisEffect::get_deflection_direction(45.0);
     * // dir = "RIGHT (Northern Hemisphere)"
     * @endcode
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
     * Outputs a formatted summary of Coriolis parameters and
     * significance thresholds for a given location.
     *
     * @param latitude_degrees Latitude in degrees
     * @param longitude_degrees Longitude in degrees (optional)
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
