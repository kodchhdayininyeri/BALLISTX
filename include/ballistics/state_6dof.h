#ifndef BALLISTX_STATE_6DOF_H
#define BALLISTX_STATE_6DOF_H

#include "utils/vec3.h"
#include "utils/quaternion.h"
#include <array>
#include <iostream>
#include <iomanip>
#include <cmath>

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ballistx {

/**
 * @brief 6-Degree-of-Freedom (6-DOF) state vector for rigid body dynamics
 *
 * Complete state representation for full 6-DOF simulation including:
 * - 3D position and linear velocity
 * - 3D orientation (quaternion, no gimbal lock)
 * - 3D angular velocity
 *
 * **State Vector Layout (13 components):**
 * ```
 * [0-2]   Position (x, y, z)           [m]
 * [3-5]   Linear velocity (vx, vy, vz) [m/s]
 * [6-9]   Orientation quaternion (qw, qx, qy, qz) [unitless]
 * [10-12] Angular velocity (wx, wy, wz) [rad/s]
 * ```
 *
 * **Applications:**
 * - Spinning artillery shells (Magnus effect)
 * - Missile guidance and control
 * - Aircraft and spacecraft attitude dynamics
 * - Full aerodynamic modeling
 *
 * **Advantages over simple position/velocity:**
 * - Accounts for spin-stabilization
 * - Enables lift and side-force modeling
 * - Supports body-frame aerodynamics
 * - Allows for control surface effects
 *
 * @example
 * @code
 * // Create initial state for artillery shell
 * Vec3 position(0.0, 0.0, 0.0);
 * Vec3 velocity(300.0, 0.0, 0.0);  // 300 m/s
 * Quaternion orientation = Quaternion::identity();
 * Vec3 angular_velocity(0.0, 0.0, 1000.0);  // 1000 rad/s spin
 *
 * State6DOF state(position, velocity, orientation, angular_velocity);
 *
 * // Or use factory method
 * State6DOF shell = State6DOF::artillery_shell(
 *     Vec3(0.0, 0.0, 0.0),
 *     Vec3(300.0, 100.0, 0.0),
 *     10000.0  // 10000 RPM spin
 * );
 *
 * // Access components
 * Vec3 pos = state.get_position();
 * double speed = state.speed();
 * Vec3 forward = state.forward();
 *
 * // Integrate with RK4
 * auto derivative = [](const State6DOF& s, double t) -> State6DOF {
 *     // Calculate derivatives
 *     State6DOF ds;
 *     ds.set_position(s.get_velocity());
 *     // ... calculate accelerations
 *     return ds;
 * };
 * @endcode
 *
 * @see Vec3 for 3D vector operations
 * @see Quaternion for orientation representation
 * @see RK4Integrator for numerical integration
 */
class State6DOF {
public:
    /**
     * @brief Size of state vector
     */
    static constexpr size_t SIZE = 13;

    /**
     * @brief State vector component indices
     *
     * Provides symbolic names for accessing state vector components.
     */
    enum Index {
        // Position
        POS_X = 0,  ///< X position
        POS_Y = 1,  ///< Y position (altitude)
        POS_Z = 2,  ///< Z position

        // Linear velocity
        VEL_X = 3,  ///< X velocity
        VEL_Y = 4,  ///< Y velocity (vertical)
        VEL_Z = 5,  ///< Z velocity

        // Orientation quaternion
        QUAT_W = 6,  ///< Quaternion scalar part
        QUAT_X = 7,  ///< Quaternion X component
        QUAT_Y = 8,  ///< Quaternion Y component
        QUAT_Z = 9,  ///< Quaternion Z component

        // Angular velocity
        ANG_VEL_X = 10,  ///< X angular velocity
        ANG_VEL_Y = 11,  ///< Y angular velocity
        ANG_VEL_Z = 12   ///< Z angular velocity
    };

    // === CONSTRUCTORS ===

    /**
     * @brief Default constructor
     *
     * Creates a zero state with identity quaternion.
     * Position and velocity are zero, orientation is identity.
     */
    State6DOF() {
        state_.fill(0.0);
        state_[QUAT_W] = 1.0;  // Identity quaternion
    }

    /**
     * @brief Create state from components
     *
     * Initializes all state components from individual vectors.
     *
     * @param position Initial position [m]
     * @param velocity Initial velocity [m/s]
     * @param orientation Initial orientation quaternion
     * @param angular_velocity Initial angular velocity [rad/s]
     *
     * @example
     * @code
     * State6DOF state(
     *     Vec3(100.0, 500.0, 0.0),      // position
     *     Vec3(200.0, 50.0, 0.0),       // velocity
     *     Quaternion::identity(),       // orientation
     *     Vec3(0.0, 0.0, 500.0)         // angular velocity
     * );
     * @endcode
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
     * @brief Create from raw state array
     *
     * Creates a state directly from a 13-element array.
     *
     * @param state Raw state array
     */
    explicit State6DOF(const std::array<double, SIZE>& state)
        : state_(state) {}

    // === POSITION ACCESSORS ===

    /**
     * @brief Get position vector
     *
     * @return Position [m]
     */
    Vec3 get_position() const {
        return Vec3(state_[POS_X], state_[POS_Y], state_[POS_Z]);
    }

    /**
     * @brief Set position vector
     *
     * @param pos Position [m]
     */
    void set_position(const Vec3& pos) {
        state_[POS_X] = pos.x;
        state_[POS_Y] = pos.y;
        state_[POS_Z] = pos.z;
    }

    /**
     * @brief Get X position component
     */
    double x() const { return state_[POS_X]; }

    /**
     * @brief Get Y position component (altitude)
     */
    double y() const { return state_[POS_Y]; }

    /**
     * @brief Get Z position component
     */
    double z() const { return state_[POS_Z]; }

    // === LINEAR VELOCITY ACCESSORS ===

    /**
     * @brief Get velocity vector
     *
     * @return Velocity [m/s]
     */
    Vec3 get_velocity() const {
        return Vec3(state_[VEL_X], state_[VEL_Y], state_[VEL_Z]);
    }

    /**
     * @brief Set velocity vector
     *
     * @param vel Velocity [m/s]
     */
    void set_velocity(const Vec3& vel) {
        state_[VEL_X] = vel.x;
        state_[VEL_Y] = vel.y;
        state_[VEL_Z] = vel.z;
    }

    /**
     * @brief Get X velocity component
     */
    double vx() const { return state_[VEL_X]; }

    /**
     * @brief Get Y velocity component (vertical)
     */
    double vy() const { return state_[VEL_Y]; }

    /**
     * @brief Get Z velocity component
     */
    double vz() const { return state_[VEL_Z]; }

    /**
     * @brief Get speed (velocity magnitude)
     *
     * @return Speed [m/s]
     */
    double speed() const {
        return get_velocity().magnitude();
    }

    // === ORIENTATION ACCESSORS ===

    /**
     * @brief Get orientation quaternion
     *
     * @return Orientation as unit quaternion
     */
    Quaternion get_orientation() const {
        return Quaternion(state_[QUAT_W],
                         state_[QUAT_X],
                         state_[QUAT_Y],
                         state_[QUAT_Z]);
    }

    /**
     * @brief Set orientation quaternion
     *
     * @param q Orientation quaternion
     */
    void set_orientation(const Quaternion& q) {
        state_[QUAT_W] = q.w;
        state_[QUAT_X] = q.x;
        state_[QUAT_Y] = q.y;
        state_[QUAT_Z] = q.z;
    }

    /**
     * @brief Get quaternion W component
     */
    double qw() const { return state_[QUAT_W]; }

    /**
     * @brief Get quaternion X component
     */
    double qx() const { return state_[QUAT_X]; }

    /**
     * @brief Get quaternion Y component
     */
    double qy() const { return state_[QUAT_Y]; }

    /**
     * @brief Get quaternion Z component
     */
    double qz() const { return state_[QUAT_Z]; }

    /**
     * @brief Get forward direction vector
     *
     * Returns the transformed Z axis based on orientation.
     * Useful for getting the projectile's forward direction.
     *
     * @return Forward direction (unit vector)
     */
    Vec3 forward() const { return get_orientation().forward(); }

    /**
     * @brief Get up direction vector
     *
     * Returns the transformed Y axis based on orientation.
     *
     * @return Up direction (unit vector)
     */
    Vec3 up() const { return get_orientation().up(); }

    /**
     * @brief Get right direction vector
     *
     * Returns the transformed X axis based on orientation.
     *
     * @return Right direction (unit vector)
     */
    Vec3 right() const { return get_orientation().right(); }

    // === ANGULAR VELOCITY ACCESSORS ===

    /**
     * @brief Get angular velocity vector
     *
     * @return Angular velocity [rad/s]
     */
    Vec3 get_angular_velocity() const {
        return Vec3(state_[ANG_VEL_X],
                    state_[ANG_VEL_Y],
                    state_[ANG_VEL_Z]);
    }

    /**
     * @brief Set angular velocity vector
     *
     * @param ang_vel Angular velocity [rad/s]
     */
    void set_angular_velocity(const Vec3& ang_vel) {
        state_[ANG_VEL_X] = ang_vel.x;
        state_[ANG_VEL_Y] = ang_vel.y;
        state_[ANG_VEL_Z] = ang_vel.z;
    }

    /**
     * @brief Get X angular velocity component
     */
    double wx() const { return state_[ANG_VEL_X]; }

    /**
     * @brief Get Y angular velocity component
     */
    double wy() const { return state_[ANG_VEL_Y]; }

    /**
     * @brief Get Z angular velocity component
     */
    double wz() const { return state_[ANG_VEL_Z]; }

    /**
     * @brief Get angular speed (angular velocity magnitude)
     *
     * @return Angular speed [rad/s]
     */
    double angular_speed() const {
        return get_angular_velocity().magnitude();
    }

    // === DERIVED QUANTITIES ===

    /**
     * @brief Get body-relative velocity
     *
     * Transforms world-frame velocity to body-frame using orientation.
     * Useful for aerodynamic calculations in body coordinates.
     *
     * @return Velocity in body frame [m/s]
     */
    Vec3 get_body_velocity() const {
        Quaternion q = get_orientation();
        Vec3 world_vel = get_velocity();
        Quaternion q_vel(0.0, world_vel.x, world_vel.y, world_vel.z);
        Quaternion q_body = q.conjugate() * q_vel * q;
        return Vec3(q_body.x, q_body.y, q_body.z);
    }

    /**
     * @brief Calculate total kinetic energy
     *
     * Returns the sum of linear and rotational kinetic energy.
     * KE = ½mv² + ½ω·I·ω
     *
     * @param mass Mass [kg]
     * @param inertia Principal moments of inertia (Ix, Iy, Iz) [kg·m²]
     * @return Kinetic energy [J]
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
     * Returns the linear momentum: p = m × v
     *
     * @param mass Mass [kg]
     * @return Momentum vector [kg·m/s]
     */
    Vec3 get_momentum(double mass) const {
        return get_velocity() * mass;
    }

    /**
     * @brief Calculate angular momentum
     *
     * Returns the angular momentum: L = I × ω
     *
     * @param inertia Principal moments of inertia (Ix, Iy, Iz) [kg·m²]
     * @return Angular momentum vector [kg·m²/s]
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
     *
     * Returns the underlying 13-element state array.
     * Useful for integration algorithms.
     *
     * @return State array
     */
    const std::array<double, SIZE>& get_state() const {
        return state_;
    }

    /**
     * @brief Set raw state array
     *
     * Sets the entire state from a 13-element array.
     *
     * @param state State array
     */
    void set_state(const std::array<double, SIZE>& state) {
        state_ = state;
    }

    /**
     * @brief Access individual component (read-only)
     *
     * @param index Component index (0-12)
     * @return Component value
     */
    double operator[](size_t index) const {
        return state_[index];
    }

    /**
     * @brief Access individual component (read-write)
     *
     * @param index Component index (0-12)
     * @return Component reference
     */
    double& operator[](size_t index) {
        return state_[index];
    }

    /**
     * @brief Add two states
     *
     * Component-wise addition. Used in RK4 integration.
     *
     * @param other State to add
     * @return Sum state
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
     *
     * Component-wise subtraction.
     *
     * @param other State to subtract
     * @return Difference state
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
     *
     * Multiplies all components by a scalar.
     *
     * @param scalar Scaling factor
     * @return Scaled state
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
     *
     * Adds another state to this one in-place.
     *
     * @param other State to add
     * @return Reference to this state
     */
    State6DOF& operator+=(const State6DOF& other) {
        for (size_t i = 0; i < SIZE; ++i) {
            state_[i] += other.state_[i];
        }
        return *this;
    }

    /**
     * @brief Compound assignment: scale
     *
     * Scales this state in-place.
     *
     * @param scalar Scaling factor
     * @return Reference to this state
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
     * Normalizes the orientation quaternion to ensure it remains a valid rotation.
     * Critical for numerical stability during integration.
     *
     * @example
     * @code
     * state.normalize_orientation();  // Call after integration step
     * @endcode
     */
    void normalize_orientation() {
        Quaternion q = get_orientation();
        q.normalize();
        set_orientation(q);
    }

    /**
     * @brief Check if state is valid
     *
     * Validates that all components are finite and the quaternion is non-zero.
     *
     * @return true if valid, false otherwise
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
     *
     * Creates a state suitable for a spin-stabilized artillery shell.
     * The projectile is aligned with its velocity direction and spinning
     * around the forward axis.
     *
     * @param position Initial position [m]
     * @param velocity Initial velocity [m/s]
     * @param spin_rate_rpm Spin rate in RPM
     * @return Initialized state
     *
     * @example
     * @code
     * State6DOF shell = State6DOF::artillery_shell(
     *     Vec3(0.0, 0.0, 0.0),    // gun position
     *     Vec3(300.0, 150.0, 0.0), // velocity
     *     12000.0                 // 12000 RPM spin
     * );
     * @endcode
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

    /**
     * @brief Stream output operator
     *
     * Outputs state in human-readable format.
     *
     * @param os Output stream
     * @param state State to output
     * @return Output stream
     */
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
     *
     * Outputs all state components on a single line.
     *
     * @param os Output stream (default: cout)
     */
    void print_compact(std::ostream& os = std::cout) const {
        os << std::fixed << std::setprecision(2);
        os << "Pos: (" << x() << ", " << y() << ", " << z() << ") ";
        os << "Vel: (" << vx() << ", " << vy() << ", " << vz() << ") ";
        os << "Quat: (" << qw() << ", " << qx() << ", " << qy() << ", " << qz() << ") ";
        os << "Ang: (" << wx() << ", " << wy() << ", " << wz() << ")";
    }

private:
    std::array<double, SIZE> state_;  ///< State vector array
};

/**
 * @brief Scalar multiplication (left side)
 *
 * Allows scalar × state multiplication.
 *
 * @param scalar Scalar value
 * @param state State to scale
 * @return Scaled state
 */
inline State6DOF operator*(double scalar, const State6DOF& state) {
    return state * scalar;
}

} // namespace ballistx

#endif // BALLISTX_STATE_6DOF_H
