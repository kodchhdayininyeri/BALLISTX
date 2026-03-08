#ifndef BALLISTX_PROPORTIONAL_NAVIGATION_H
#define BALLISTX_PROPORTIONAL_NAVIGATION_H

#include "guidance/guidance.h"
#include <cmath>

namespace ballistx {

/**
 * @brief Proportional Navigation (PN) Guidance Law
 *
 * The most widely used guidance law for homing missiles.
 *
 * Principle: Acceleration command is proportional to line-of-sight (LOS) rate.
 *
 * Command: a_cmd = N * V_c * ω_los
 *
 * Where:
 * - N = Navigation gain (typically 3-5)
 * - V_c = Closing velocity [m/s]
 * - ω_los = Line-of-sight rotation rate [rad/s]
 *
 * Physics:
 * - Maintains constant LOS bearing to target
 * - Results in intercept course for constant-velocity targets
 * - Optimal gain N = 3 for minimal control effort
 *
 * Variants:
 * - Pure PN (PPN): Command perpendicular to velocity
 * - True PN (TPN): Command perpendicular to LOS
 * - Augmented PN (APN): Accounts for target maneuvering
 */
class ProportionalNavigation : public Guidance {
public:
    /**
     * @brief PN variant type
     */
    enum class Variant {
        PURE_PN,    // Command perpendicular to missile velocity (most common)
        TRUE_PN,    // Command perpendicular to LOS
        AUGMENTED_PN // Accounts for target acceleration
    };

    /**
     * @brief Constructor
     *
     * @param nav_gain Navigation gain N (dimensionless)
     *                 - Typical: 3.0 (optimal for non-maneuvering targets)
     *                 - Range: 2.0-6.0 (higher = more aggressive, more overshoot)
     * @param variant PN variant to use
     */
    explicit ProportionalNavigation(double nav_gain = 3.0, Variant variant = Variant::PURE_PN)
        : nav_gain_(nav_gain)
        , variant_(variant)
        , max_accel_(1000.0) // Default max lateral acceleration [m/s²]
    {
        if (nav_gain_ < 1.0) {
            nav_gain_ = 3.0; // Minimum gain for stability
        }
    }

    /**
     * @brief Set maximum lateral acceleration [m/s²]
     *
     * Real missiles have limited maneuver capability (structural/aerodynamic limits)
     */
    void set_max_acceleration(double max_accel) {
        max_accel_ = std::max(1.0, max_accel);
    }

    /**
     * @brief Set navigation gain
     */
    void set_navigation_gain(double gain) {
        nav_gain_ = std::max(1.0, gain);
    }

    /**
     * @brief Set PN variant
     */
    void set_variant(Variant variant) {
        variant_ = variant;
    }

    // ========================================================================
    // Guidance Interface Implementation
    // ========================================================================

    /**
     * @brief Calculate PN guidance command
     */
    GuidanceCommand calculate_command(const State6DOF& state, const Target& target) override {
        GuidanceCommand cmd;

        // Get relative state
        Vec3 los_vec = target.position - state.get_position();
        double range = los_vec.magnitude();

        // Check for target proximity
        if (range < proximity_threshold_) {
            cmd.detonation_command = true;
            cmd.time_to_go = 0.0;
            cmd.status_msg = "Target proximity - fuse armed";
            return cmd;
        }

        Vec3 los_dir = los_vec.normalized();
        Vec3 rel_vel = target.velocity - state.get_velocity();

        // Calculate closing velocity
        double closing_vel = -rel_vel.dot(los_dir);

        // Check if closing on target
        if (closing_vel < 0.0) {
            cmd.is_valid = false;
            cmd.status_msg = "Opening - target receding";
            return cmd;
        }

        // Calculate LOS rate (angular velocity of LOS vector)
        Vec3 los_rate = calculate_los_rate_vector(state, target);

        // Calculate acceleration command based on variant
        Vec3 accel_cmd;
        switch (variant_) {
            case Variant::PURE_PN:
                accel_cmd = calculate_pure_pn(state, closing_vel, los_rate);
                break;

            case Variant::TRUE_PN:
                accel_cmd = calculate_true_pn(closing_vel, los_rate, los_dir);
                break;

            case Variant::AUGMENTED_PN:
                accel_cmd = calculate_augmented_pn(state, target, closing_vel, los_rate, los_dir);
                break;
        }

        // Clamp to maximum acceleration
        double accel_mag = accel_cmd.magnitude();
        if (accel_mag > max_accel_) {
            accel_cmd = accel_cmd.normalized() * max_accel_;
            cmd.status_msg = "Acceleration limited";
        }

        // Set command
        cmd.acceleration_command = accel_cmd;
        cmd.time_to_go = estimate_time_to_go(state, target);
        cmd.is_valid = true;

        return cmd;
    }

    /**
     * @brief Check if engagement is feasible
     *
     * PN requires:
     * - Target within seeker field-of-view (if seeker-limited)
     * - Positive closing velocity
     * - Sufficient time-to-go for maneuver
     */
    bool can_engage(const State6DOF& state, const Target& target) const override {
        Vec3 los = target.position - state.get_position();
        double range = los.magnitude();

        // Too close - no time to maneuver
        if (range < minimum_engagement_range_) {
            return false;
        }

        // Check closing velocity
        Vec3 los_dir = los.normalized();
        Vec3 rel_vel = target.velocity - state.get_velocity();
        double closing_vel = -rel_vel.dot(los_dir);

        if (closing_vel < minimum_closing_velocity_) {
            return false;
        }

        // Check field-of-view constraint (if limited)
        if (has_fov_limit_) {
            Vec3 missile_dir = state.get_velocity().normalized();
            double angle = std::acos(std::max(-1.0, std::min(1.0,
                missile_dir.dot(los_dir))));

            if (angle > max_fov_angle_) {
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Estimate time-to-go for PN
     *
     * Uses range divided by closing velocity (simplified)
     * More accurate methods exist but this is sufficient for guidance
     */
    double estimate_time_to_go(const State6DOF& state, const Target& target) const override {
        Vec3 los = target.position - state.get_position();
        Vec3 los_dir = los.normalized();
        Vec3 rel_vel = state.get_velocity() - target.velocity;

        double closing_vel = rel_vel.dot(los_dir);
        double range = los.magnitude();

        if (closing_vel < 1e-6) {
            return -1.0;
        }

        return range / closing_vel;
    }

    void reset() override {
        // Pure PN has no internal state
    }

    std::string get_name() const override {
        switch (variant_) {
            case Variant::PURE_PN: return "Pure Proportional Navigation";
            case Variant::TRUE_PN: return "True Proportional Navigation";
            case Variant::AUGMENTED_PN: return "Augmented Proportional Navigation";
        }
        return "Proportional Navigation";
    }

    std::string get_description() const override {
        return "Acceleration proportional to LOS rate. Most common homing guidance law.";
    }

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set seeker field-of-view limit
     *
     * Many missiles have limited seeker FOV. PN only works if target
     * remains within this cone.
     *
     * @param max_angle_deg Maximum off-boresight angle [degrees]
     *                      Set to -1 to disable FOV limit (default)
     */
    void set_fov_limit(double max_angle_deg) {
        if (max_angle_deg < 0.0) {
            has_fov_limit_ = false;
        } else {
            has_fov_limit_ = true;
            max_fov_angle_ = max_angle_deg * M_PI / 180.0;
        }
    }

    /**
     * @brief Set minimum engagement range [m]
     *
     * Below this range, guidance is deactivated (fuse takes over)
     */
    void set_minimum_engagement_range(double range) {
        minimum_engagement_range_ = std::max(0.0, range);
    }

    /**
     * @brief Set proximity fuse threshold [m]
     *
     * Distance at which detonation command is issued
     */
    void set_proximity_threshold(double distance) {
        proximity_threshold_ = std::max(0.0, distance);
    }

private:
    // Parameters
    double nav_gain_;                // Navigation gain N (typically 3-5)
    Variant variant_;                // PN variant
    double max_accel_;               // Maximum lateral acceleration [m/s²]

    // Constraints
    bool has_fov_limit_ = false;     // Whether FOV is limited
    double max_fov_angle_ = M_PI / 3.0; // Max off-boresight angle [rad]
    double minimum_engagement_range_ = 100.0; // Minimum range for guidance [m]
    double minimum_closing_velocity_ = 10.0;  // Minimum closing speed [m/s]
    double proximity_threshold_ = 5.0;         // Proximity fuse distance [m]

    // ========================================================================
    // PN Variant Implementations
    // ========================================================================

    /**
     * @brief Pure PN: Command perpendicular to missile velocity
     *
     * a_cmd = N * V_c * (ω_los × V_missile_dir)
     *
     * Most common variant. Used in most modern missiles.
     */
    Vec3 calculate_pure_pn(const State6DOF& state, double closing_vel, const Vec3& los_rate) const {
        Vec3 vel_dir = state.get_velocity().normalized();

        // Cross product gives direction perpendicular to both LOS rate and velocity
        // This ensures acceleration is lateral (thrust assumed along velocity)
        Vec3 cmd_dir = vel_dir.cross(los_rate);

        // If LOS rate is parallel to velocity (edge case), use alternate method
        if (cmd_dir.magnitude_squared() < 1e-12) {
            cmd_dir = los_rate;
        }

        // Magnitude: N * V_c * |ω_los|
        double cmd_mag = nav_gain_ * closing_vel * los_rate.magnitude();

        return cmd_dir.normalized() * cmd_mag;
    }

    /**
     * @brief True PN: Command perpendicular to LOS
     *
     * a_cmd = N * V_c * ω_los
     *
     * Simpler but less practical (harder to implement in real missiles)
     */
    Vec3 calculate_true_pn(double closing_vel, const Vec3& los_rate, const Vec3& los_dir) const {
        // Command is directly in the direction perpendicular to LOS
        // Project LOS rate onto plane perpendicular to LOS
        Vec3 los_rate_perp = los_rate - los_dir * (los_rate.dot(los_dir));

        double cmd_mag = nav_gain_ * closing_vel * los_rate_perp.magnitude();
        return los_rate_perp.normalized() * cmd_mag;
    }

    /**
     * @brief Augmented PN: Accounts for target maneuver
     *
     * a_cmd = N * V_c * ω_los + (N/2) * a_target_perp
     *
     * Better performance against maneuvering targets
     * Requires estimate of target acceleration
     */
    Vec3 calculate_augmented_pn(const State6DOF& state, const Target& target,
                                double closing_vel, const Vec3& los_rate, const Vec3& los_dir) const {
        // Base PN command
        Vec3 base_cmd = calculate_pure_pn(state, closing_vel, los_rate);

        // Add target acceleration term
        // Only consider target acceleration perpendicular to LOS
        Vec3 target_accel_perp = target.acceleration -
            los_dir * (target.acceleration.dot(los_dir));

        Vec3 augmented_cmd = base_cmd + target_accel_perp * (nav_gain_ / 2.0);

        return augmented_cmd;
    }

    /**
     * @brief Calculate LOS rate vector
     *
     * ω_los = (R × V_rel) / |R|²
     *
     * Where R is relative position, V_rel is relative velocity
     */
    Vec3 calculate_los_rate_vector(const State6DOF& state, const Target& target) const {
        Vec3 los = target.position - state.get_position();
        Vec3 rel_vel = target.velocity - state.get_velocity();

        double range_sq = los.magnitude_squared();
        if (range_sq < 1e-12) {
            return Vec3::zero();
        }

        // ω = (R × V) / R²
        return los.cross(rel_vel) / range_sq;
    }
};

/**
 * @brief Pure Pursuit Guidance
 *
 * Simplest guidance law: always point directly at target.
 *
 * Command: Accelerate toward current target position (no lead).
 *
 * Use case:
 * - Very short range engagements
 * - Stationary or slow-moving targets
 * - When implementation simplicity is critical
 *
 * Limitations:
 * - inefficient against moving targets (tail chase)
 * - No interception, just pursuit
 */
class PurePursuit : public Guidance {
public:
    explicit PurePursuit(double max_accel = 1000.0)
        : max_accel_(max_accel)
    {}

    GuidanceCommand calculate_command(const State6DOF& state, const Target& target) override {
        GuidanceCommand cmd;

        // Direction to target
        Vec3 los = target.position - state.get_position();
        double range = los.magnitude();

        if (range < proximity_threshold_) {
            cmd.detonation_command = true;
            cmd.time_to_go = 0.0;
            cmd.status_msg = "Target proximity";
            return cmd;
        }

        Vec3 los_dir = los.normalized();

        // Command: max acceleration toward target
        cmd.acceleration_command = los_dir * max_accel_;
        cmd.time_to_go = estimate_time_to_go(state, target);
        cmd.is_valid = true;

        return cmd;
    }

    double estimate_time_to_go(const State6DOF& state, const Target& target) const override {
        double range = (target.position - state.get_position()).magnitude();
        double speed = state.get_velocity().magnitude();
        return (speed > 1e-6) ? range / speed : -1.0;
    }

    std::string get_name() const override { return "Pure Pursuit"; }
    std::string get_description() const override {
        return "Always point at target. Simple but inefficient.";
    }

    void set_proximity_threshold(double dist) { proximity_threshold_ = dist; }
    void set_max_acceleration(double accel) { max_accel_ = accel; }

private:
    double max_accel_ = 1000.0;
    double proximity_threshold_ = 5.0;
};

} // namespace ballistx

#endif // BALLISTX_PROPORTIONAL_NAVIGATION_H
