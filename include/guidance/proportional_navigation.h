#ifndef BALLISTX_PROPORTIONAL_NAVIGATION_H
#define BALLISTX_PROPORTIONAL_NAVIGATION_H

#include "guidance/guidance.h"
#include <cmath>
#include <algorithm>

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ballistx {

/**
 * @brief Proportional Navigation (PN) Guidance Law
 *
 * The most widely used guidance law for homing missiles.
 *
 * ============================================================================
 * PRINCIPLE
 * ============================================================================
 *
 * Acceleration command is proportional to line-of-sight (LOS) rotation rate.
 *
 *                a_cmd = N * V_c * ω_los
 *
 * Where:
 *   N      = Navigation gain (typically 3-5, dimensionless)
 *   V_c    = Closing velocity [m/s] (positive when approaching)
 *   ω_los  = Line-of-sight angular rate [rad/s]
 *
 * ============================================================================
 * PHYSICS
 * ============================================================================
 *
 * - Maintains constant LOS bearing to target (parallel navigation)
 * - Results in intercept course for constant-velocity targets
 * - Optimal gain N = 3 for minimal control effort against non-maneuvering targets
 * - Higher N reduces miss distance but increases acceleration demand
 *
 * ============================================================================
 * VARIANTS
 * ============================================================================
 *
 * 1. Pure PN (PPN)        - Command perpendicular to missile velocity vector
 * 2. True PN (TPN)        - Command perpendicular to LOS vector
 * 3. Augmented PN (APN)   - Accounts for target maneuver (acceleration term)
 * 4. Ideal PN (IPN)      - Theoretical optimal, requires future knowledge
 * 5. Biased PN           - Modified gain for specific scenarios
 *
 * ============================================================================
 * NAVIGATION GAIN SELECTION
 * ============================================================================
 *
 *   N = 2.0  : Minimum for stability (slow response)
 *   N = 3.0  : Optimal for non-maneuvering targets (most common)
 *   N = 4-5  : Against maneuvering targets
 *   N > 5    : Aggressive, may cause oscillation
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 *
 * [1] Zarchan, P. "Tactical and Strategic Missile Guidance"
 * [2] Shneydor, N.A. "Missile Guidance and Pursuit"
 * [3] Yanushevsky, R. "Modern Missile Guidance"
 */
class ProportionalNavigation : public Guidance {
public:
    /**
     * @brief PN variant type
     */
    enum class Variant {
        PURE_PN,        ///< Command perpendicular to missile velocity (most common)
        TRUE_PN,        ///< Command perpendicular to LOS (theoretical)
        AUGMENTED_PN,   ///< With target acceleration compensation
        IDEAL_PN,       ///< Optimal but requires integration
        BIASED_PN       ///< Variable gain based on engagement geometry
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

    // ========================================================================
    // CONFIGURATION METHODS
    // ========================================================================

    /**
     * @brief Set maximum lateral acceleration [m/s²]
     *
     * Real missiles have limited maneuver capability due to:
     * - Structural limits (airframe can't take excessive G)
     * - Aerodynamic limits (fin stall at high AoA)
     * - Actuator limits (fin deflection limits)
     */
    void set_max_acceleration(double max_accel) {
        max_accel_ = std::max(1.0, max_accel);
    }

    /**
     * @brief Set navigation gain N
     *
     * Can be adjusted in-flight for adaptive guidance
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

    /**
     * @brief Enable/disable adaptive gain (for Biased PN)
     *
     * When enabled, gain varies based on time-to-go:
     * - Higher gain at long range (coarse correction)
     * - Lower gain at short range (fine adjustment)
     */
    void set_adaptive_gain(bool enable, double min_gain = 2.0, double max_gain = 6.0) {
        adaptive_gain_enabled_ = enable;
        adaptive_min_gain_ = min_gain;
        adaptive_max_gain_ = max_gain;
    }

    /**
     * @brief Set g-bias (constant acceleration command)
     *
     * Used in some implementations to compensate for gravity or
     * to maintain minimum G-load for structural reasons.
     */
    void set_g_bias(double bias_g) {
        g_bias_ = bias_g * 9.81; // Convert to m/s²
    }

    // ========================================================================
    // GUIDANCE INTERFACE IMPLEMENTATION
    // ========================================================================

    /**
     * @brief Calculate PN guidance command
     *
     * Core guidance law implementation
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
        if (closing_vel < minimum_closing_velocity_) {
            cmd.is_valid = false;
            cmd.status_msg = "Opening - target receding";
            return cmd;
        }

        // Estimate time-to-go
        double time_to_go = (closing_vel > 1e-6) ? range / closing_vel : -1.0;
        cmd.time_to_go = time_to_go;

        // Calculate LOS rate (angular velocity of LOS vector)
        Vec3 los_rate = calculate_los_rate_vector(state, target);
        double los_rate_mag = los_rate.magnitude();

        // Adaptive gain calculation
        double effective_gain = nav_gain_;
        if (adaptive_gain_enabled_ && time_to_go > 0.0) {
            effective_gain = calculate_adaptive_gain(time_to_go, los_rate_mag);
        }

        // Calculate acceleration command based on variant
        Vec3 accel_cmd;
        switch (variant_) {
            case Variant::PURE_PN:
                accel_cmd = calculate_pure_pn(state, closing_vel, los_rate, effective_gain);
                break;

            case Variant::TRUE_PN:
                accel_cmd = calculate_true_pn(closing_vel, los_rate, los_dir, effective_gain);
                break;

            case Variant::AUGMENTED_PN:
                accel_cmd = calculate_augmented_pn(state, target, closing_vel,
                                                   los_rate, los_dir, effective_gain);
                break;

            case Variant::IDEAL_PN:
                accel_cmd = calculate_ideal_pn(state, target, range, closing_vel,
                                               los_rate, time_to_go, effective_gain);
                break;

            case Variant::BIASED_PN:
                accel_cmd = calculate_biased_pn(state, closing_vel, los_rate,
                                                los_dir, effective_gain);
                break;
        }

        // Add G-bias if configured
        if (std::abs(g_bias_) > 1e-6) {
            accel_cmd.y += g_bias_; // Assume Y is up
        }

        // Clamp to maximum acceleration
        double accel_mag = accel_cmd.magnitude();
        if (accel_mag > max_accel_) {
            accel_cmd = accel_cmd.normalized() * max_accel_;
            cmd.status_msg = "Acceleration limited to " +
                std::to_string(static_cast<int>(max_accel_ / 9.81)) + "G";
        }

        // Set command
        cmd.acceleration_command = accel_cmd;
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
        // Pure PN has no internal state to reset
    }

    std::string get_name() const override {
        switch (variant_) {
            case Variant::PURE_PN: return "Pure Proportional Navigation (PPN)";
            case Variant::TRUE_PN: return "True Proportional Navigation (TPN)";
            case Variant::AUGMENTED_PN: return "Augmented Proportional Navigation (APN)";
            case Variant::IDEAL_PN: return "Ideal Proportional Navigation (IPN)";
            case Variant::BIASED_PN: return "Biased Proportional Navigation";
        }
        return "Proportional Navigation";
    }

    std::string get_description() const override {
        return "Acceleration command: a = N * Vc * ω_los. "
               "The classic homing guidance law used in most missiles.";
    }

    // ========================================================================
    // DIAGNOSTICS
    // ========================================================================

    /**
     * @brief Get current navigation parameters (for debugging/telemetry)
     */
    struct Telemetry {
        double navigation_gain;
        double closing_velocity;
        double los_rate_magnitude;
        double time_to_go;
        double commanded_acceleration;
        double off_boresight_angle;
        bool acceleration_limited;
    };

    Telemetry get_telemetry(const State6DOF& state, const Target& target) const {
        Telemetry tel;

        Vec3 los = target.position - state.get_position();
        Vec3 los_dir = los.normalized();
        Vec3 rel_vel = state.get_velocity() - target.velocity;

        tel.navigation_gain = nav_gain_;
        tel.closing_velocity = rel_vel.dot(los_dir);
        tel.los_rate_magnitude = calculate_los_rate_vector(state, target).magnitude();
        tel.time_to_go = estimate_time_to_go(state, target);

        Vec3 missile_dir = state.get_velocity().normalized();
        tel.off_boresight_angle = std::acos(std::max(-1.0, std::min(1.0,
            missile_dir.dot(los_dir)))) * 180.0 / M_PI;

        // Estimate commanded acceleration
        double accel_mag = nav_gain_ * tel.closing_velocity * tel.los_rate_magnitude;
        tel.commanded_acceleration = accel_mag;
        tel.acceleration_limited = (accel_mag > max_accel_);

        return tel;
    }

private:
    // ========================================================================
    // PARAMETERS
    // ========================================================================

    double nav_gain_;                // Navigation gain N (typically 3-5)
    Variant variant_;                // PN variant
    double max_accel_;               // Maximum lateral acceleration [m/s²]

    // Constraints
    bool has_fov_limit_ = false;     // Whether FOV is limited
    double max_fov_angle_ = M_PI / 3.0; // Max off-boresight angle [rad]
    double minimum_engagement_range_ = 50.0; // Minimum range for guidance [m]
    double minimum_closing_velocity_ = 1.0;  // Minimum closing speed [m/s] (relaxed)
    double proximity_threshold_ = 50.0;        // Proximity fuse distance [m] (increased)

    // Advanced options
    bool adaptive_gain_enabled_ = false;
    double adaptive_min_gain_ = 2.0;
    double adaptive_max_gain_ = 6.0;
    double g_bias_ = 0.0;  // Constant G-bias [m/s²]

    // ========================================================================
    // PN VARIANT IMPLEMENTATIONS
    // ========================================================================

    /**
     * @brief Pure PN: Command perpendicular to missile velocity
     *
     * Formula: a_cmd = N * V_c * |ω_los| * n_direction
     *
     * Where n_direction is perpendicular to velocity vector.
     *
     * Most common variant. Used in:
     * - AIM-9 Sidewinder
     * - AIM-120 AMRAAM
     * - Most modern missiles
     *
     * Advantages:
     * - Easy to implement (acceleration naturally perpendicular to velocity)
     * - Works well with thrust-aligned missiles
     *
     * Disadvantages:
     * - Not theoretically optimal
     * - Can produce large demands at high aspect angles
     */
    Vec3 calculate_pure_pn(const State6DOF& state, double closing_vel,
                          const Vec3& los_rate, double effective_gain) const {
        Vec3 vel_dir = state.get_velocity().normalized();

        // Cross product gives direction perpendicular to both LOS rate and velocity
        // This ensures acceleration is lateral (thrust assumed along velocity)
        Vec3 cmd_dir = vel_dir.cross(los_rate);

        // If LOS rate is parallel to velocity (edge case), use alternate method
        if (cmd_dir.magnitude_squared() < 1e-12) {
            cmd_dir = los_rate;
        }

        // Magnitude: N * V_c * |ω_los|
        double cmd_mag = effective_gain * closing_vel * los_rate.magnitude();

        return cmd_dir.normalized() * cmd_mag;
    }

    /**
     * @brief True PN: Command perpendicular to LOS
     *
     * Formula: a_cmd = N * V_c * ω_los_perp
     *
     * Where ω_los_perp is LOS rate projected onto plane perpendicular to LOS.
     *
     * Theoretical advantage:
     * - Maintains constant LOS rate exactly
     * - More mathematically elegant
     *
     * Practical issues:
     * - Harder to implement (require acceleration not aligned with velocity)
     * - Requires thrust vectoring or very high angle-of-attack
     * - Rarely used in practice
     *
     * Mainly of academic interest.
     */
    Vec3 calculate_true_pn(double closing_vel, const Vec3& los_rate,
                          const Vec3& los_dir, double effective_gain) const {
        // Command is directly in the direction perpendicular to LOS
        // Project LOS rate onto plane perpendicular to LOS
        Vec3 los_rate_perp = los_rate - los_dir * (los_rate.dot(los_dir));

        double cmd_mag = effective_gain * closing_vel * los_rate_perp.magnitude();

        if (los_rate_perp.magnitude() < 1e-12) {
            return Vec3::zero();
        }

        return los_rate_perp.normalized() * cmd_mag;
    }

    /**
     * @brief Augmented PN: Accounts for target maneuver
     *
     * Formula: a_cmd = N * V_c * ω_los + (N/2) * a_target_perp
     *
     * Additional term compensates for target acceleration perpendicular to LOS.
     *
     * Used against:
     * - Maneuvering aircraft (evasive maneuvers)
     * - Ballistic missiles (during powered phase)
     *
     * Advantages:
     * - Significantly reduces miss distance against maneuvering targets
     * - Requires similar G-load as target (efficient)
     *
     * Disadvantages:
     * - Requires target acceleration estimate (needs seeker with Doppler)
     * - Degrades if acceleration estimate is noisy
     *
     * Used in:
     * - AIM-54 Phoenix (early versions)
     * - Some surface-to-air missiles
     */
    Vec3 calculate_augmented_pn(const State6DOF& state, const Target& target,
                                double closing_vel, const Vec3& los_rate,
                                const Vec3& los_dir, double effective_gain) const {
        // Base PN command
        Vec3 base_cmd = calculate_pure_pn(state, closing_vel, los_rate, effective_gain);

        // Add target acceleration term
        // Only consider target acceleration perpendicular to LOS
        Vec3 target_accel_perp = target.acceleration -
            los_dir * (target.acceleration.dot(los_dir));

        Vec3 augmented_cmd = base_cmd + target_accel_perp * (effective_gain / 2.0);

        return augmented_cmd;
    }

    /**
     * @brief Ideal PN: Theoretical optimal guidance
     *
     * Formula: a_cmd = N * V_c * ω_los + N * (a_target - a_missile) / 2
     *
     * Additional terms account for both target and missile acceleration.
     * Requires integration of acceleration over time.
     *
     * Theoretical properties:
     * - Zero miss distance for constant maneuvering target
     * - Optimal in terms of control effort
     *
     * Practical issues:
     * - Requires future knowledge (not physically realizable)
     * - Needs accurate acceleration estimation for both bodies
     *
     * Mainly used as:
     * - Benchmark for comparing other laws
     * - Theoretical analysis
     *
     * Note: This implementation is simplified approximation.
     * True IPN requires solving a two-point boundary value problem.
     */
    Vec3 calculate_ideal_pn(const State6DOF& state, const Target& target,
                           double range, double closing_vel, const Vec3& los_rate,
                           double time_to_go, double effective_gain) const {
        if (time_to_go < 0.0) {
            return Vec3::zero();
        }

        // Base PN command
        Vec3 base_cmd = calculate_pure_pn(state, closing_vel, los_rate, effective_gain);

        // Target acceleration term
        Vec3 target_accel_perp = target.acceleration;
        double los_dot_target = (target.position - state.get_position()).normalized()
            .dot(target_accel_perp);
        target_accel_perp = target_accel_perp - (target.position - state.get_position())
            .normalized() * los_dot_target;

        // Ideal correction: compensate for target maneuver over remaining time
        // This is simplified - full IPN requires integration
        Vec3 ideal_correction = target_accel_perp * (effective_gain / 2.0);

        return base_cmd + ideal_correction;
    }

    /**
     * @brief Biased PN: Variable gain based on engagement
     *
     * Adjusts navigation gain based on flight conditions:
     * - Higher gain early (when LOS rate is noisy)
     * - Lower gain late (to avoid over-correction)
     * - Adds constant bias for gravity compensation
     *
     * Use cases:
     * - Surface-to-air missiles (gravity compensation)
     * - Long-range engagements (adaptive tuning)
     * - When seeker noise is significant at long range
     */
    Vec3 calculate_biased_pn(const State6DOF& state, double closing_vel,
                            const Vec3& los_rate, const Vec3& los_dir,
                            double effective_gain) const {
        // Calculate base PN
        Vec3 base_cmd = calculate_pure_pn(state, closing_vel, los_rate, effective_gain);

        // Add gravity bias (assume Y is up)
        Vec3 gravity_compensation(0.0, g_bias_, 0.0);

        return base_cmd + gravity_compensation;
    }

    /**
     * @brief Calculate adaptive gain based on time-to-go
     *
     * Gain schedule:
     * - t_go > 10s: Use max_gain (aggressive correction)
     * - 5s < t_go < 10s: Linear interpolation
     * - t_go < 5s: Use min_gain (fine adjustment, avoid overshoot)
     */
    double calculate_adaptive_gain(double time_to_go, double los_rate_mag) const {
        if (!adaptive_gain_enabled_) {
            return nav_gain_;
        }

        // Simple gain scheduling based on time-to-go
        double gain;
        if (time_to_go > 10.0) {
            gain = adaptive_max_gain_;
        } else if (time_to_go < 5.0) {
            gain = adaptive_min_gain_;
        } else {
            // Linear interpolation
            double alpha = (time_to_go - 5.0) / 5.0;
            gain = adaptive_min_gain_ + alpha * (adaptive_max_gain_ - adaptive_min_gain_);
        }

        // Also consider LOS rate - reduce gain if LOS rate is very small
        // (to avoid amplifying noise)
        if (los_rate_mag < 1e-6) {
            gain = std::min(gain, 2.0);
        }

        return gain;
    }

    /**
     * @brief Calculate LOS rate vector
     *
     * Formula: ω_los = (R × V_rel) / |R|²
     *
     * Where:
     *   R      = Relative position vector (target - missile)
     *   V_rel  = Relative velocity vector (target_vel - missile_vel)
     *
     * This is the angular rate at which the LOS vector rotates.
     * Higher LOS rate means sharper turn required.
     *
     * LOS rate is zero on a collision course (理想的拦截航线)
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
 * Use cases:
 * - Very short range engagements (< 2 km)
 * - Stationary or slow-moving targets
 * - When implementation simplicity is critical
 * - Anti-tank missiles (some variants)
 *
 * Limitations:
 * - Inefficient against moving targets (tail chase required)
 * - No true interception, just pursuit
 * - High energy loss (always trailing target)
 * - Vulnerable to target decoys
 *
 * Comparison to PN:
 * - PN leads the target (intercept course)
 * - Pure pursuit trails the target (pursuit course)
 * - PN is typically 3-5x more efficient in energy usage
 *
 * Mathematical relation:
 * Pure pursuit = PN with N = 1 (or approaching 1 at tail chase)
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

        // Check if still engaging
        if (range > max_engagement_range_) {
            cmd.is_valid = false;
            cmd.status_msg = "Target out of range";
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

    bool can_engage(const State6DOF& state, const Target& target) const override {
        double range = (target.position - state.get_position()).magnitude();
        return range < max_engagement_range_ && range > minimum_range_;
    }

    std::string get_name() const override { return "Pure Pursuit"; }
    std::string get_description() const override {
        return "Always point at target. Simple but inefficient against moving targets. "
               "Best for very short range or stationary targets.";
    }

    void set_proximity_threshold(double dist) { proximity_threshold_ = dist; }
    void set_max_acceleration(double accel) { max_accel_ = accel; }
    void set_max_engagement_range(double range) { max_engagement_range_ = range; }
    void set_minimum_range(double range) { minimum_range_ = range; }

private:
    double max_accel_ = 1000.0;
    double proximity_threshold_ = 50.0;  // Increased for demo
    double max_engagement_range_ = 10000.0;
    double minimum_range_ = 50.0;
};

} // namespace ballistx

#endif // BALLISTX_PROPORTIONAL_NAVIGATION_H
