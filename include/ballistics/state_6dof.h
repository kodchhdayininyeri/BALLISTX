#ifndef BALLISTX_STATE_6DOF_H
#define BALLISTX_STATE_6DOF_H

#include "utils/vec3.h"
#include "utils/quaternion.h"
#include <array>
#include <iostream>
#include <cmath>

namespace ballistx {

/**
 * @brief 6-Degree-of-Freedom State Vector
 *
 * Complete 13-dimensional state for rigid body dynamics:
 *
 * [0-2]   Position (x, y, z)        - meters
 * [3-5]   Linear velocity (vx, vy, vz) - m/s
 * [6-9]   Orientation quaternion (qw, qx, qy, qz) - unit quaternion
 * [10-12] Angular velocity (wx, wy, wz) - rad/s
 *
 * This enables full 6-DOF simulation including:
 * - 3D position and velocity
 * - 3D orientation (no gimbal lock)
 * - 3D angular velocity
 * - Magnus effect (spinning projectiles)
 * - Full aerodynamic modeling
 */
class State6DOF {
public:
    // State vector size
    static constexpr size_t SIZE = 13;

    // Component indices
    enum Index {
        // Position
        POS_X = 0,
        POS_Y = 1,
        POS_Z = 2,

        // Linear velocity
        VEL_X = 3,
        VEL_Y = 4,
        VEL_Z = 5,

        // Orientation quaternion
        QUAT_W = 6,
        QUAT_X = 7,
        QUAT_Y = 8,
        QUAT_Z = 9,

        // Angular velocity
        ANG_VEL_X = 10,
        ANG_VEL_Y = 11,
        ANG_VEL_Z = 12
    };

    // === CONSTRUCTORS ===

    State6DOF() {
        state_.fill(0.0);
        state_[QUAT_W] = 1.0;  // Identity quaternion
    }

    /**
     * @brief Create state from components
     */
    State6DOF(const Vec3& position,
              const Vec3& velocity,
              const Quaternion& orientation,
              const Vec3& angular_velocity) {
        set_position(position);
        set_velocity(velocity);
        set_orientation(orientation);
        set_angular_velocity(angular_velocity);
    }

    /**
     * @brief Create from raw array
     */
    explicit State6DOF(const std::array<double, SIZE>& state)
        : state_(state) {}

    // === POSITION ACCESSORS ===

    Vec3 get_position() const {
        return Vec3(state_[POS_X], state_[POS_Y], state_[POS_Z]);
    }

    void set_position(const Vec3& pos) {
        state_[POS_X] = pos.x;
        state_[POS_Y] = pos.y;
        state_[POS_Z] = pos.z;
    }

    double x() const { return state_[POS_X]; }
    double y() const { return state_[POS_Y]; }
    double z() const { return state_[POS_Z]; }

    // === LINEAR VELOCITY ACCESSORS ===

    Vec3 get_velocity() const {
        return Vec3(state_[VEL_X], state_[VEL_Y], state_[VEL_Z]);
    }

    void set_velocity(const Vec3& vel) {
        state_[VEL_X] = vel.x;
        state_[VEL_Y] = vel.y;
        state_[VEL_Z] = vel.z;
    }

    double vx() const { return state_[VEL_X]; }
    double vy() const { return state_[VEL_Y]; }
    double vz() const { return state_[VEL_Z]; }

    double speed() const {
        return get_velocity().magnitude();
    }

    // === ORIENTATION ACCESSORS ===

    Quaternion get_orientation() const {
        return Quaternion(state_[QUAT_W],
                         state_[QUAT_X],
                         state_[QUAT_Y],
                         state_[QUAT_Z]);
    }

    void set_orientation(const Quaternion& q) {
        state_[QUAT_W] = q.w;
        state_[QUAT_X] = q.x;
        state_[QUAT_Y] = q.y;
        state_[QUAT_Z] = q.z;
    }

    double qw() const { return state_[QUAT_W]; }
    double qx() const { return state_[QUAT_X]; }
    double qy() const { return state_[QUAT_Y]; }
    double qz() const { return state_[QUAT_Z]; }

    /**
     * @brief Get direction vectors from orientation
     */
    Vec3 forward() const { return get_orientation().forward(); }
    Vec3 up() const { return get_orientation().up(); }
    Vec3 right() const { return get_orientation().right(); }

    // === ANGULAR VELOCITY ACCESSORS ===

    Vec3 get_angular_velocity() const {
        return Vec3(state_[ANG_VEL_X],
                    state_[ANG_VEL_Y],
                    state_[ANG_VEL_Z]);
    }

    void set_angular_velocity(const Vec3& ang_vel) {
        state_[ANG_VEL_X] = ang_vel.x;
        state_[ANG_VEL_Y] = ang_vel.y;
        state_[ANG_VEL_Z] = ang_vel.z;
    }

    double wx() const { return state_[ANG_VEL_X]; }
    double wy() const { return state_[ANG_VEL_Y]; }
    double wz() const { return state_[ANG_VEL_Z]; }

    double angular_speed() const {
        return get_angular_velocity().magnitude();
    }

    // === DERIVED QUANTITIES ===

    /**
     * @brief Get body-relative velocity
     *
     * Transforms world velocity to body frame using orientation
     */
    Vec3 get_body_velocity() const {
        Quaternion q = get_orientation();
        Vec3 world_vel = get_velocity();
        Quaternion q_vel(0.0, world_vel.x, world_vel.y, world_vel.z);
        Quaternion q_body = q.conjugate() * q_vel * q;
        return Vec3(q_body.x, q_body.y, q_body.z);
    }

    /**
     * @brief Calculate kinetic energy
     *
     * KE = 0.5 * m * v² + 0.5 * ω · I · ω
     */
    double get_kinetic_energy(double mass, const Vec3& inertia) const {
        Vec3 vel = get_velocity();
        Vec3 ang_vel = get_angular_velocity();

        double linear_ke = 0.5 * mass * vel.magnitude_squared();
        double angular_ke = 0.5 * (ang_vel.x * ang_vel.x * inertia.x +
                                  ang_vel.y * ang_vel.y * inertia.y +
                                  ang_vel.z * ang_vel.z * inertia.z);

        return linear_ke + angular_ke;
    }

    /**
     * @brief Calculate linear momentum
     *
     * p = m * v
     */
    Vec3 get_momentum(double mass) const {
        return get_velocity() * mass;
    }

    /**
     * @brief Calculate angular momentum
     *
     * L = I * ω
     */
    Vec3 get_angular_momentum(const Vec3& inertia) const {
        Vec3 ang_vel = get_angular_velocity();
        return Vec3(ang_vel.x * inertia.x,
                   ang_vel.y * inertia.y,
                   ang_vel.z * inertia.z);
    }

    // === STATE VECTOR OPERATIONS ===

    /**
     * @brief Get raw state array
     */
    const std::array<double, SIZE>& get_state() const {
        return state_;
    }

    /**
     * @brief Set raw state array
     */
    void set_state(const std::array<double, SIZE>& state) {
        state_ = state;
    }

    /**
     * @brief Access individual component
     */
    double operator[](size_t index) const {
        return state_[index];
    }

    double& operator[](size_t index) {
        return state_[index];
    }

    /**
     * @brief Add two states (for RK4 integration)
     */
    State6DOF operator+(const State6DOF& other) const {
        State6DOF result;
        for (size_t i = 0; i < SIZE; ++i) {
            result.state_[i] = state_[i] + other.state_[i];
        }
        return result;
    }

    /**
     * @brief Subtract two states
     */
    State6DOF operator-(const State6DOF& other) const {
        State6DOF result;
        for (size_t i = 0; i < SIZE; ++i) {
            result.state_[i] = state_[i] - other.state_[i];
        }
        return result;
    }

    /**
     * @brief Scale state by scalar
     */
    State6DOF operator*(double scalar) const {
        State6DOF result;
        for (size_t i = 0; i < SIZE; ++i) {
            result.state_[i] = state_[i] * scalar;
        }
        return result;
    }

    /**
     * @brief Compound assignment: add
     */
    State6DOF& operator+=(const State6DOF& other) {
        for (size_t i = 0; i < SIZE; ++i) {
            state_[i] += other.state_[i];
        }
        return *this;
    }

    /**
     * @brief Compound assignment: scale
     */
    State6DOF& operator*=(double scalar) {
        for (size_t i = 0; i < SIZE; ++i) {
            state_[i] *= scalar;
        }
        return *this;
    }

    // === NORMALIZATION ===

    /**
     * @brief Normalize quaternion component
     *
     * Critical for numerical stability during integration
     */
    void normalize_orientation() {
        Quaternion q = get_orientation();
        q.normalize();
        set_orientation(q);
    }

    /**
     * @brief Check if state is valid
     */
    bool is_valid() const {
        // Check position is finite
        Vec3 pos = get_position();
        if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) {
            return false;
        }

        // Check velocity is finite
        Vec3 vel = get_velocity();
        if (!std::isfinite(vel.x) || !std::isfinite(vel.y) || !std::isfinite(vel.z)) {
            return false;
        }

        // Check quaternion is valid (non-zero magnitude)
        Quaternion q = get_orientation();
        if (q.magnitude_squared() < 1e-16) {
            return false;
        }

        // Check angular velocity is finite
        Vec3 ang_vel = get_angular_velocity();
        if (!std::isfinite(ang_vel.x) || !std::isfinite(ang_vel.y) || !std::isfinite(ang_vel.z)) {
            return false;
        }

        return true;
    }

    // === FACTORY METHODS ===

    /**
     * @brief Create initial state for artillery shell
     */
    static State6DOF artillery_shell(const Vec3& position,
                                     const Vec3& velocity,
                                     double spin_rate_rpm) {
        // Convert RPM to rad/s
        double spin_rate = spin_rate_rpm * 2.0 * M_PI / 60.0;

        // For artillery, spin is around the velocity vector (forward direction)
        Vec3 forward_dir = velocity.normalized();
        Vec3 angular_velocity = forward_dir * spin_rate;

        // Orientation: aligned with velocity
        // Calculate quaternion from velocity direction
        Vec3 up(0.0, 1.0, 0.0);
        Vec3 right = forward_dir.cross(up).normalized();
        if (right.magnitude_squared() < 1e-6) {
            // Velocity is nearly vertical
            right = Vec3(1.0, 0.0, 0.0);
        }
        up = right.cross(forward_dir).normalized();

        // Create rotation matrix from basis vectors
        double matrix[9] = {
            right.x, up.x, forward_dir.x,
            right.y, up.y, forward_dir.y,
            right.z, up.z, forward_dir.z
        };

        // Convert to quaternion (simplified)
        Quaternion q = Quaternion::from_axis_angle(up, 0.0);

        return State6DOF(position, velocity, q, angular_velocity);
    }

    // === STREAM OUTPUT ===

    friend std::ostream& operator<<(std::ostream& os, const State6DOF& state) {
        os << "State6DOF:\n";
        os << "  Position:        " << state.get_position() << "\n";
        os << "  Velocity:        " << state.get_velocity() << " (speed: "
           << state.speed() << " m/s)\n";
        os << "  Orientation:     " << state.get_orientation() << "\n";
        os << "  Angular Vel:     " << state.get_angular_velocity() << " ("
           << (state.angular_speed() * 60.0 / (2.0 * M_PI)) << " RPM)\n";
        return os;
    }

    /**
     * @brief Print state in compact format
     */
    void print_compact(std::ostream& os = std::cout) const {
        os << std::fixed << std::setprecision(2);
        os << "Pos: (" << x() << ", " << y() << ", " << z() << ") ";
        os << "Vel: (" << vx() << ", " << vy() << ", " << vz() << ") ";
        os << "Quat: (" << qw() << ", " << qx() << ", " << qy() << ", " << qz() << ") ";
        os << "Ang: (" << wx() << ", " << wy() << ", " << wz() << ")";
    }

private:
    std::array<double, SIZE> state_;
};

/**
 * @brief Scalar multiplication (left side)
 */
inline State6DOF operator*(double scalar, const State6DOF& state) {
    return state * scalar;
}

} // namespace ballistx

#endif // BALLISTX_STATE_6DOF_H
