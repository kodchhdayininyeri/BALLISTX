#ifndef BALLISTX_GUIDANCE_H
#define BALLISTX_GUIDANCE_H

#include "utils/vec3.h"
#include "ballistics/state_6dof.h"
#include <memory>
#include <string>

namespace ballistx {

/**
 * @brief Target information for guidance algorithms
 *
 * Represents the goal state that the guidance system tries to reach.
 * Can be a stationary target, moving target, or predicted intercept point.
 */
struct Target {
    Vec3 position;           // Target position [m]
    Vec3 velocity;           // Target velocity [m/s] (zero for stationary)
    Vec3 acceleration;       // Target acceleration [m/s²] (optional)

    // Target metadata
    bool is_moving() const { return velocity.magnitude_squared() > 1e-12; }
    bool has_prediction() const { return acceleration.magnitude_squared() > 1e-12; }

    Target() = default;
    Target(Vec3 pos) : position(pos), velocity(Vec3::zero()), acceleration(Vec3::zero()) {}
    Target(Vec3 pos, Vec3 vel) : position(pos), velocity(vel), acceleration(Vec3::zero()) {}
    Target(Vec3 pos, Vec3 vel, Vec3 acc) : position(pos), velocity(vel), acceleration(acc) {}

    /**
     * @brief Predict target position at future time
     * @param time_ahead Time to predict ahead [s]
     * @return Predicted position assuming constant acceleration
     */
    Vec3 predict_position(double time_ahead) const {
        // p(t) = p0 + v0*t + 0.5*a*t²
        return position + velocity * time_ahead + acceleration * (0.5 * time_ahead * time_ahead);
    }

    /**
     * @brief Predict target velocity at future time
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
 * Represents the control command calculated by guidance algorithm.
 * Units are normalized (0-1) for different control surfaces/effects.
 */
struct GuidanceCommand {
    // Lateral acceleration command [m/s²] (direction + magnitude)
    Vec3 acceleration_command;

    // Body-axis control commands (normalized -1 to 1)
    double pitch_command;    // Elevator/Canard (-1: full down, +1: full up)
    double yaw_command;      // Rudder (-1: full left, +1: full right)
    double roll_command;     // Ailerons (-1: full left, +1: full right)
    double throttle_command; // Throttle (0: idle, 1: full)

    // Auxiliary
    bool detonation_command; // Fuse trigger (for proximity/impact fuses)
    double time_to_go;       // Estimated time to intercept [s]

    // Guidance metadata
    bool is_valid;           // Whether command is valid
    std::string status_msg;  // Status/error message

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
     */
    static GuidanceCommand invalid(const std::string& msg) {
        GuidanceCommand cmd;
        cmd.is_valid = false;
        cmd.status_msg = msg;
        return cmd;
    }

    /**
     * @brief Normalize acceleration command to body-axis controls
     * @param max_lateral_accel Maximum lateral acceleration capability [m/s²]
     * @param max_axial_accel Maximum axial (thrust/brake) acceleration [m/s²]
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
 * Guidance algorithms calculate the required acceleration/controls
 * to steer a projectile toward a target.
 *
 * Subclasses implement specific guidance laws:
 * - Proportional Navigation (PN)
 * - Pure Pursuit
 * - Lead Pursuit
 * - Predictive Guidance
 * - Optimal Guidance
 *
 * Usage:
 *   auto guidance = std::make_unique<ProportionalNavigation>(nav_gain);
 *   GuidanceCommand cmd = guidance->calculate_command(state, target);
 */
class Guidance {
public:
    virtual ~Guidance() = default;

    /**
     * @brief Calculate guidance command for current state and target
     *
     * @param state Current projectile state (position, velocity, orientation)
     * @param target Target to engage
     * @return Guidance command (acceleration and/or control surface commands)
     *
     * This is the main interface - all guidance algorithms must implement this.
     * Called each time step during simulation.
     */
    virtual GuidanceCommand calculate_command(const State6DOF& state, const Target& target) = 0;

    /**
     * @brief Reset guidance state (for multi-target scenarios)
     *
     * Some guidance algorithms maintain internal state (e.g., integrated errors).
     * This method resets that state for a new engagement.
     */
    virtual void reset() {
        // Default: nothing to reset
    }

    /**
     * @brief Check if guidance can still engage target
     *
     * @param state Current projectile state
     * @param target Target to check
     * @return true if engagement is still feasible
     *
     * Examples where guidance might fail:
     * - Target out of range (field-of-view limited seeker)
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
     * @brief Estimate time to intercept [s]
     *
     * @param state Current projectile state
     * @param target Target to engage
     * @return Estimated time to impact (seconds), or negative if not calculable
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
     */
    virtual std::string get_name() const = 0;

    /**
     * @brief Get guidance algorithm type description
     */
    virtual std::string get_description() const {
        return "Base guidance algorithm";
    }

protected:
    // Helper: calculate line-of-sight (LOS) vector to target
    Vec3 calculate_los(const State6DOF& state, const Target& target) const {
        return (target.position - state.get_position()).normalized();
    }

    // Helper: calculate line-of-sight rate (derivative of LOS direction)
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

    // Helper: calculate closing velocity
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
 */
class GuidanceFactory {
public:
    /**
     * @brief Create guidance algorithm by type name
     *
     * Supported types (as subclasses are implemented):
     * - "pn" or "proportional_navigation"
     * - "pure_pursuit"
     * - "lead_pursuit"
     * - "predictive"
     *
     * @param type Guidance algorithm type
     * @param params Optional parameters (e.g., navigation gain)
     * @return Guidance algorithm instance
     */
    static GuidancePtr create(const std::string& type, double param = 0.0);
};

} // namespace ballistx

#endif // BALLISTX_GUIDANCE_H
