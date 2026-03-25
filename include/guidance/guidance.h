#ifndef BALLISTX_GUIDANCE_H
#define BALLISTX_GUIDANCE_H

#include "utils/vec3.h"
#include "ballistics/state_6dof.h"
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

namespace ballistx {

/**
 * @brief Target information for guidance algorithms
 *
 * Represents the goal state that guidance systems try to intercept.
 * Supports stationary targets, moving targets, and targets with
 * acceleration for predictive guidance.
 *
 * **Coordinate System:**
 * - Position: Target location in world coordinates [m]
 * - Velocity: Target velocity in world coordinates [m/s]
 * - Acceleration: Target acceleration in world coordinates [m/s²]
 *
 * @example
 * @code
 * // Stationary target
 * Target stationary(Vec3(1000.0, 0.0, 0.0));
 *
 * // Moving target
 * Target moving(Vec3(1000.0, 0.0, 0.0), Vec3(50.0, 0.0, 0.0));
 *
 * // Maneuvering target
 * Target maneuvering(
 *     Vec3(5000.0, 3000.0, 0.0),
 *     Vec3(200.0, 0.0, 0.0),
 *     Vec3(5.0, 0.0, 0.0)  // 5 m/s² acceleration
 * );
 *
 * // Predict future position
 * Vec3 future_pos = target.predict_position(2.0);  // 2 seconds ahead
 * @endcode
 *
 * @see Guidance for guidance algorithms
 * @see ProportionalNavigation for PN implementation
 */
struct Target {
    Vec3 position;       ///< Target position [m]
    Vec3 velocity;       ///< Target velocity [m/s] (zero for stationary)
    Vec3 acceleration;   ///< Target acceleration [m/s²] (optional)

    /**
     * @brief Check if target is moving
     *
     * @return true if velocity magnitude is significant
     */
    bool is_moving() const { return velocity.magnitude_squared() > 1e-12; }

    /**
     * @brief Check if target has acceleration for prediction
     *
     * @return true if acceleration magnitude is significant
     */
    bool has_prediction() const { return acceleration.magnitude_squared() > 1e-12; }

    /**
     * @brief Default constructor - creates target at origin
     */
    Target() = default;

    /**
     * @brief Create stationary target
     *
     * @param pos Target position [m]
     */
    Target(Vec3 pos) : position(pos), velocity(Vec3::zero()), acceleration(Vec3::zero()) {}

    /**
     * @brief Create moving target with constant velocity
     *
     * @param pos Target position [m]
     * @param vel Target velocity [m/s]
     */
    Target(Vec3 pos, Vec3 vel) : position(pos), velocity(vel), acceleration(Vec3::zero()) {}

    /**
     * @brief Create maneuvering target with acceleration
     *
     * @param pos Target position [m]
     * @param vel Target velocity [m/s]
     * @param acc Target acceleration [m/s²]
     */
    Target(Vec3 pos, Vec3 vel, Vec3 acc) : position(pos), velocity(vel), acceleration(acc) {}

    /**
     * @brief Predict target position at future time
     *
     * Uses kinematic equation: p(t) = p₀ + v₀×t + 0.5×a×t²
     *
     * @param time_ahead Time to predict ahead [s]
     * @return Predicted position assuming constant acceleration
     *
     * @example
     * @code
     * Target target(Vec3(0.0, 0.0, 0.0), Vec3(100.0, 0.0, 0.0));
     * Vec3 pos_1s = target.predict_position(1.0);  // (100, 0, 0)
     * Vec3 pos_2s = target.predict_position(2.0);  // (200, 0, 0)
     * @endcode
     */
    Vec3 predict_position(double time_ahead) const {
        // p(t) = p0 + v0*t + 0.5*a*t²
        return position + velocity * time_ahead + acceleration * (0.5 * time_ahead * time_ahead);
    }

    /**
     * @brief Predict target velocity at future time
     *
     * Uses: v(t) = v₀ + a×t
     *
     * @param time_ahead Time to predict ahead [s]
     * @return Predicted velocity assuming constant acceleration
     */
    Vec3 predict_velocity(double time_ahead) const {
        // v(t) = v0 + a*t
        return velocity + acceleration * time_ahead;
    }
};

/**
 * @brief Guidance command output
 *
 * Contains the control commands calculated by guidance algorithms.
 * Includes acceleration commands and control surface deflections.
 *
 * **Command Types:**
 * - Acceleration command: World-frame lateral acceleration [m/s²]
 * - Body-axis controls: Normalized (-1 to +1) for control surfaces
 * - Auxiliary: Detonation trigger, time-to-go estimate
 *
 * @example
 * @code
 * GuidanceCommand cmd;
 * cmd.acceleration_command = Vec3(0.0, 20.0, 5.0);  // Lateral accel
 * cmd.pitch_command = 0.5;    // 50% up elevator
 * cmd.yaw_command = -0.2;     // 20% left rudder
 * cmd.detonation_command = false;
 * cmd.time_to_go = 2.5;       // 2.5 seconds to intercept
 * cmd.is_valid = true;
 * @endcode
 *
 * @see Guidance for command generation
 */
struct GuidanceCommand {
    // Lateral acceleration command
    Vec3 acceleration_command;  ///< Lateral acceleration [m/s²]

    // Body-axis control commands (normalized -1 to 1)
    double pitch_command;     ///< Elevator/Canard (-1: full down, +1: full up)
    double yaw_command;       ///< Rudder (-1: full left, +1: full right)
    double roll_command;      ///< Ailerons (-1: full left, +1: full right)
    double throttle_command;  ///< Throttle (0: idle, 1: full)

    // Auxiliary commands
    bool detonation_command;  ///< Fuse trigger (for proximity/impact fuses)
    double time_to_go;        ///< Estimated time to intercept [s]

    // Guidance metadata
    bool is_valid;            ///< Whether command is valid
    std::string status_msg;   ///< Status/error message

    /**
     * @brief Default constructor - creates zero command
     *
     * Initializes all commands to zero/neutral with valid status.
     */
    GuidanceCommand()
        : acceleration_command(Vec3::zero())
        , pitch_command(0.0)
        , yaw_command(0.0)
        , roll_command(0.0)
        , throttle_command(0.0)
        , detonation_command(false)
        , time_to_go(0.0)
        , is_valid(true)
        , status_msg("OK")
    {}

    /**
     * @brief Create invalid command with error message
     *
     * @param msg Error message describing why command is invalid
     * @return Invalid guidance command
     *
     * @example
     * @code
     * auto cmd = GuidanceCommand::invalid("Target out of range");
     * @endcode
     */
    static GuidanceCommand invalid(const std::string& msg) {
        GuidanceCommand cmd;
        cmd.is_valid = false;
        cmd.status_msg = msg;
        return cmd;
    }

    /**
     * @brief Normalize acceleration command to body-axis controls
     *
     * Converts world-frame acceleration to normalized control commands.
     * Simplified conversion that assumes X-axis is forward.
     *
     * @param max_lateral_accel Maximum lateral acceleration [m/s²]
     * @param max_axial_accel Maximum axial acceleration [m/s²]
     */
    void normalize_to_controls(double max_lateral_accel, double max_axial_accel) {
        // Convert world acceleration to body frame
        // This is a simplified conversion - full implementation would use orientation
        double lateral_mag = acceleration_command.magnitude();
        double axial_mag = acceleration_command.x; // Assume x is forward

        // Clamp to [-1, 1]
        pitch_command = std::max(-1.0, std::min(1.0, acceleration_command.y / max_lateral_accel));
        yaw_command = std::max(-1.0, std::min(1.0, acceleration_command.z / max_lateral_accel));
        throttle_command = std::max(0.0, std::min(1.0, axial_mag / max_axial_accel));
    }
};

/**
 * @brief Abstract base class for guidance algorithms
 *
 * Defines the interface for guidance systems that calculate control
 * commands to steer projectiles toward targets.
 *
 * **Guidance Laws:**
 * - Proportional Navigation (PN): Most widely used for missiles
 * - Pure Pursuit: Always points directly at target
 * - Lead Pursuit: Aims ahead of moving target
 * - Predictive Guidance: Uses target prediction
 * - Optimal Guidance: Minimizes control effort / miss distance
 *
 * **Usage Pattern:**
 * ```cpp
 * auto guidance = std::make_unique<ProportionalNavigation>(3.0);
 *
 * while (simulating) {
 *     GuidanceCommand cmd = guidance->calculate_command(state, target);
 *     if (cmd.is_valid) {
 *         apply_control(cmd);
 *     }
 * }
 * ```
 *
 * @example
 * @code
 * // Create PN guidance with N=3
 * ProportionalNavigation pn(3.0);
 *
 * // Calculate command for current state
 * State6DOF state = ...;
 * Target target = ...;
 * GuidanceCommand cmd = pn.calculate_command(state, target);
 *
 * // Apply command
 * if (cmd.is_valid) {
 *     missile.apply_acceleration(cmd.acceleration_command);
 * }
 * @endcode
 *
 * @see ProportionalNavigation for PN implementation
 * @see Target for target representation
 * @see GuidanceCommand for command format
 */
class Guidance {
public:
    virtual ~Guidance() = default;

    /**
     * @brief Calculate guidance command for current state and target
     *
     * This is the main interface that all guidance algorithms must implement.
     * Called each time step during simulation.
     *
     * @param state Current projectile state (position, velocity, orientation)
     * @param target Target to engage
     * @return Guidance command (acceleration and/or control surface commands)
     */
    virtual GuidanceCommand calculate_command(const State6DOF& state, const Target& target) = 0;

    /**
     * @brief Reset guidance state for new engagement
     *
     * Some guidance algorithms maintain internal state (e.g., integrated errors).
     * This method resets that state for a new target engagement.
     */
    virtual void reset() {
        // Default: nothing to reset
    }

    /**
     * @brief Check if guidance can still engage target
     *
     * Determines if engagement is still feasible based on current conditions.
     *
     * @param state Current projectile state
     * @param target Target to check
     * @return true if engagement is still feasible
     *
     * **Failure conditions:**
     * - Target out of seeker field-of-view
     * - Insufficient energy to maneuver
     * - Time-to-go too small
     */
    virtual bool can_engage(const State6DOF& state, const Target& target) const {
        // Default: always engage
        (void)state;
        (void)target;
        return true;
    }

    /**
     * @brief Estimate time to intercept
     *
     * Calculates estimated time until impact with target.
     * Default implementation uses closing velocity method.
     *
     * @param state Current projectile state
     * @param target Target to engage
     * @return Estimated time to impact [s], or negative if not calculable
     */
    virtual double estimate_time_to_go(const State6DOF& state, const Target& target) const {
        // Default: closing velocity method
        Vec3 rel_pos = target.position - state.get_position();
        Vec3 rel_vel = state.get_velocity() - target.velocity;

        double closing_speed = rel_vel.magnitude();
        if (closing_speed < 1e-6) {
            return -1.0; // Cannot estimate
        }

        double range = rel_pos.magnitude();
        return range / closing_speed;
    }

    /**
     * @brief Get guidance algorithm name
     *
     * @return Algorithm name as string
     */
    virtual std::string get_name() const = 0;

    /**
     * @brief Get guidance algorithm description
     *
     * @return Human-readable description
     */
    virtual std::string get_description() const {
        return "Base guidance algorithm";
    }

protected:
    /**
     * @brief Calculate line-of-sight (LOS) vector to target
     *
     * @param state Current projectile state
     * @param target Target
     * @return Normalized LOS vector
     */
    Vec3 calculate_los(const State6DOF& state, const Target& target) const {
        return (target.position - state.get_position()).normalized();
    }

    /**
     * @brief Calculate line-of-sight rate
     *
     * Computes the rate of change of LOS direction.
     *
     * @param state Current projectile state
     * @param target Target
     * @return LOS rate vector
     */
    Vec3 calculate_los_rate(const State6DOF& state, const Target& target) const {
        Vec3 los = target.position - state.get_position();
        Vec3 rel_vel = target.velocity - state.get_velocity();

        double range = los.magnitude();
        if (range < 1e-6) {
            return Vec3::zero();
        }

        // LOS rate = (V_rel x LOS) / |R|
        // This is a simplified calculation
        Vec3 los_dir = los.normalized();
        Vec3 los_rate = rel_vel - los_dir * (rel_vel.dot(los_dir));
        return los_rate / range;
    }

    /**
     * @brief Calculate closing velocity
     *
     * Returns the rate at which missile and target are closing.
     *
     * @param state Current projectile state
     * @param target Target
     * @return Closing velocity [m/s]
     */
    double calculate_closing_velocity(const State6DOF& state, const Target& target) const {
        Vec3 los = target.position - state.get_position();
        Vec3 rel_vel = state.get_velocity() - target.velocity;

        double range = los.magnitude();
        if (range < 1e-6) {
            return 0.0;
        }

        // Closing velocity = -dR/dt projected onto LOS
        return -rel_vel.dot(los.normalized());
    }
};

/**
 * @brief Shared pointer type for guidance algorithms
 */
using GuidancePtr = std::unique_ptr<Guidance>;

/**
 * @brief Factory for creating guidance algorithms
 *
 * Creates guidance algorithm instances by type name.
 * Useful for runtime algorithm selection.
 */
class GuidanceFactory {
public:
    /**
     * @brief Create guidance algorithm by type name
     *
     * Supported types:
     * - "pn" or "proportional_navigation": Proportional Navigation
     * - "pure_pursuit": Pure Pursuit guidance
     * - "lead_pursuit": Lead Pursuit guidance
     * - "predictive": Predictive guidance
     *
     * @param type Guidance algorithm type
     * @param param Optional parameter (e.g., navigation gain for PN)
     * @return Guidance algorithm instance
     */
    static GuidancePtr create(const std::string& type, double param = 0.0);
};

} // namespace ballistx

#endif // BALLISTX_GUIDANCE_H
