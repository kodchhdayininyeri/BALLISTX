#ifndef BALLISTX_MAGNUS_EFFECT_H
#define BALLISTX_MAGNUS_EFFECT_H

#include "utils/vec3.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ballistx {

/**
 * @brief Magnus effect calculations for spinning projectiles
 *
 * Models the lateral force experienced by spinning projectiles moving
 * through air. The force is perpendicular to both the spin axis and
 * velocity vector.
 *
 * **Physical Principle:**
 * - Spinning projectile drags air around it
 * - On one side, air moves WITH the flow (lower velocity, higher pressure)
 * - On opposite side, air moves AGAINST flow (higher velocity, lower pressure)
 * - Pressure difference creates lateral force (Bernoulli effect)
 *
 * **Formula:**
 * ```
 * F_magnus = S × (ω × v)
 * ```
 * where S is spin coefficient, ω is angular velocity, v is velocity
 *
 * **Applications:**
 * - Baseball curveballs and pitches
 * - Artillery shell drift correction
 * - Sniper long-range accuracy
 * - Spin-stabilized projectile stability
 *
 * @example
 * @code
 * // Calculate Magnus force on spinning artillery shell
 * Vec3 velocity(300.0, 0.0, 0.0);           // 300 m/s
 * Vec3 angular_velocity(0.0, 0.0, 1000.0);   // 1000 rad/s spin
 * double air_density = 1.225;                // sea level
 *
 * Vec3 magnus_force = MagnusEffect::calculate_force(
 *     velocity, angular_velocity, air_density, 0.3
 * );
 *
 * // Check gyroscopic stability
 * bool stable = MagnusEffect::is_stable(
 *     12000.0,    // 12000 RPM
 *     300.0,     // 300 m/s
 *     0.155,     // 155 mm diameter
 *     0.6,       // 600 mm length
 *     50.0       // 50 kg mass
 * );
 * @endcode
 *
 * @see Atmosphere for air density
 * @see State6DOF for angular velocity
 */
class MagnusEffect {
public:
    /**
     * @brief Calculate Magnus force on spinning projectile
     *
     * Returns the lateral force perpendicular to both spin axis
     * and velocity vector: F = S × ρ × (ω × v)
     *
     * @param velocity Projectile velocity [m/s]
     * @param angular_velocity Angular velocity [rad/s]
     * @param air_density Air density [kg/m³]
     * @param spin_coefficient Spin coefficient (default: 1.0)
     * @return Magnus force vector [N]
     *
     * @example
     * @code
     * Vec3 force = MagnusEffect::calculate_force(
     *     Vec3(300.0, 0.0, 0.0),
     *     Vec3(0.0, 0.0, 1000.0),
     *     1.225,
     *     0.3
     * );
     * @endcode
     */
    static Vec3 calculate_force(const Vec3& velocity,
                               const Vec3& angular_velocity,
                               double air_density,
                               double spin_coefficient = 1.0) {
        // F_magnus = S × (ω × v)
        Vec3 omega_cross_v = angular_velocity.cross(velocity);

        // Magnitude scaling
        double magnitude = spin_coefficient * air_density * omega_cross_v.magnitude();

        // Direction is perpendicular to both spin and velocity
        if (magnitude > 1e-12) {
            return omega_cross_v.normalized() * magnitude;
        }

        return Vec3(0.0, 0.0, 0.0);
    }

    /**
     * @brief Calculate Magnus force with radius-based coefficient
     *
     * More accurate formulation for specific projectile geometries.
     * F = Cm × ρ × R³ × (ω × v)
     *
     * @param velocity Projectile velocity [m/s]
     * @param angular_velocity Angular velocity [rad/s]
     * @param air_density Air density [kg/m³]
     * @param radius Projectile radius [m]
     * @param magnus_coefficient Magnus coefficient (default: 0.3)
     * @return Magnus force vector [N]
     */
    static Vec3 calculate_force_with_radius(const Vec3& velocity,
                                           const Vec3& angular_velocity,
                                           double air_density,
                                           double radius,
                                           double magnus_coefficient = 0.3) {
        // F_magnus = C_m × ρ × R³ × (ω × v)
        Vec3 omega_cross_v = angular_velocity.cross(velocity);
        double factor = magnus_coefficient * air_density * std::pow(radius, 3.0);

        return omega_cross_v * factor;
    }

    /**
     * @brief Calculate Magnus acceleration
     *
     * Returns acceleration: a = F / m
     *
     * @param velocity Projectile velocity [m/s]
     * @param angular_velocity Angular velocity [rad/s]
     * @param air_density Air density [kg/m³]
     * @param mass Projectile mass [kg]
     * @param spin_coefficient Spin coefficient (default: 1.0)
     * @return Magnus acceleration [m/s²]
     */
    static Vec3 calculate_acceleration(const Vec3& velocity,
                                      const Vec3& angular_velocity,
                                      double air_density,
                                      double mass,
                                      double spin_coefficient = 1.0) {
        Vec3 force = calculate_force(velocity, angular_velocity,
                                    air_density, spin_coefficient);
        return force / mass;
    }

    /**
     * @brief Estimate spin coefficient for projectile
     *
     * Empirical approximation based on length-to-diameter ratio.
     * Typical values range from 0.2 to 0.4 for artillery shells.
     *
     * @param length_to_diameter_ratio L/D ratio (typically 4-6 for artillery)
     * @return Estimated spin coefficient
     */
    static double estimate_spin_coefficient(double length_to_diameter_ratio) {
        // Empirical approximation for artillery shells
        // Base value around 0.3, modified by L/D ratio
        double base_coefficient = 0.3;
        double ratio_factor = length_to_diameter_ratio - 4.0;
        if (ratio_factor > 2.0) ratio_factor = 2.0;
        if (ratio_factor < -1.0) ratio_factor = -1.0;
        ratio_factor /= 4.0;

        return base_coefficient * (1.0 + ratio_factor);
    }

    /**
     * @brief Estimate lateral deflection due to Magnus effect
     *
     * Simplified calculation for long-range trajectory planning.
     * Returns estimated lateral drift at target.
     *
     * @param velocity Average velocity [m/s]
     * @param spin_rate Spin rate [RPM]
     * @param flight_time Total flight time [s]
     * @param mass Projectile mass [kg]
     * @param air_density Average air density [kg/m³]
     * @param radius Projectile radius [m]
     * @return Lateral deflection [m]
     *
     * @example
     * @code
     * double drift = MagnusEffect::estimate_lateral_deflection(
     *     250.0,   // 250 m/s average
     *     10000.0, // 10000 RPM
     *     60.0,    // 60 second flight
     *     50.0,    // 50 kg
     *     1.0,     // sea level density
     *     0.0775   // 155mm radius
     * );
     * @endcode
     */
    static double estimate_lateral_deflection(double velocity,
                                            double spin_rate,
                                            double flight_time,
                                            double mass,
                                            double air_density,
                                            double radius) {
        double omega = spin_rate * 2.0 * M_PI / 60.0;  // RPM to rad/s

        // Simplified Magnus force: F = C × ρ × R³ × ω × v
        double magnus_coeff = 0.3;
        double force_mag = magnus_coeff * air_density * std::pow(radius, 3.0) * omega * velocity;

        // Deflection: d = 0.5 × a × t²
        double acceleration = force_mag / mass;
        double deflection = 0.5 * acceleration * flight_time * flight_time;

        return deflection;
    }

    /**
     * @brief Calculate spin decay over time
     *
     * Models the reduction in spin rate due to air resistance.
     * Uses exponential decay model.
     *
     * @param initial_spin Initial spin rate [RPM]
     * @param time Flight time [s]
     * @param air_density Air density [kg/m³]
     * @param radius Projectile radius [m]
     * @param decay_coefficient Spin decay coefficient (default: 0.0001)
     * @return Current spin rate [RPM]
     */
    static double calculate_spin_decay(double initial_spin,
                                     double time,
                                     double air_density,
                                     double radius,
                                     double decay_coefficient = 0.0001) {
        // Exponential decay model: ω(t) = ω₀ × exp(-k × ρ × t)
        double decay_factor = decay_coefficient * air_density * time;
        return initial_spin * std::exp(-decay_factor);
    }

    /**
     * @brief Get Magnus force direction relative to velocity
     *
     * Returns unit vector in direction of Magnus force.
     *
     * @param spin_axis Axis of rotation (normalized)
     * @param velocity Velocity vector
     * @return Unit vector in Magnus force direction
     */
    static Vec3 get_force_direction(const Vec3& spin_axis, const Vec3& velocity) {
        Vec3 omega_cross_v = spin_axis.cross(velocity);
        if (omega_cross_v.magnitude_squared() < 1e-12) {
            return Vec3(0.0, 0.0, 0.0);
        }
        return omega_cross_v.normalized();
    }

    /**
     * @brief Calculate gyroscopic stability factor
     *
     * Returns Miller stability factor for spin-stabilized projectiles.
     * Values > 1.3 indicate stable flight.
     *
     * @param spin_rate Spin rate [RPM]
     * @param velocity Velocity [m/s]
     * @param diameter Projectile diameter [m]
     * @param length Projectile length [m]
     * @param mass Mass [kg]
     * @return Stability factor (> 1.3 for stable flight)
     *
     * @example
     * @code
     * double S = MagnusEffect::calculate_gyroscopic_stability(
     *     12000.0, 12000.0, 0.155, 0.6, 50.0
     * );
     * if (S > 1.3) {
     *     std::cout << "Projectile is stable" << std::endl;
     * }
     * @endcode
     */
    static double calculate_gyroscopic_stability(double spin_rate,
                                                double velocity,
                                                double diameter,
                                                double length,
                                                double mass) {
        // Simplified Miller stability factor
        double omega = spin_rate * 2.0 * M_PI / 60.0;  // rad/s
        double moment_of_inertia = 0.5 * mass * diameter * diameter;  // Approximation

        // S = (4 × I × ω) / (m × d² × v)
        double stability = (4.0 * moment_of_inertia * omega) /
                         (mass * diameter * diameter * velocity);

        return stability;
    }

    /**
     * @brief Check if projectile is gyroscopically stable
     *
     * Convenience function that returns true if stability factor > 1.3.
     *
     * @param spin_rate Spin rate [RPM]
     * @param velocity Velocity [m/s]
     * @param diameter Projectile diameter [m]
     * @param length Projectile length [m]
     * @param mass Mass [kg]
     * @return true if stability factor > 1.3
     */
    static bool is_stable(double spin_rate,
                        double velocity,
                        double diameter,
                        double length,
                        double mass) {
        double stability = calculate_gyroscopic_stability(spin_rate, velocity,
                                                         diameter, length, mass);
        return stability > 1.3;
    }
};

} // namespace ballistx

#endif // BALLISTX_MAGNUS_EFFECT_H
