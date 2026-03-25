#ifndef BALLISTX_QUATERNION_H
#define BALLISTX_QUATERNION_H

#include "utils/vec3.h"
#include <cmath>
#include <iostream>
#include <utility>

namespace ballistx {

/**
 * @brief Quaternion class for 3D rotations and orientation
 *
 * Quaternions represent 3D rotations as: q = w + xi + yj + zk
 * where w is the scalar part and (x, y, z) is the vector part.
 *
 * **Advantages over Euler angles:**
 * - No gimbal lock
 * - Smooth interpolation (slerp)
 * - Efficient composition of rotations
 * - Numerically stable for long simulations
 * - Compact representation (4 numbers vs 9 for rotation matrix)
 *
 * **Common uses in ballistics:**
 * - Projectile orientation and attitude
 * - Camera and sensor rotation
 * - Angular velocity integration
 * - Aerospace attitude control
 * - Transforming vectors between coordinate frames
 *
 * @example
 * @code
 * // Create rotation from Euler angles (pitch, yaw, roll)
 * Quaternion q = Quaternion::from_euler(0.1, 0.5, 0.0);  // radians
 *
 * // Rotate a vector
 * Vec3 forward(0.0, 0.0, 1.0);
 * Vec3 rotated = q.rotate(forward);
 *
 * // Compose rotations
 * Quaternion q1 = Quaternion::from_axis_angle(Vec3::up(), 0.5);
 * Quaternion q2 = Quaternion::from_axis_angle(Vec3::right(), 0.3);
 * Quaternion combined = q1 * q2;  // Apply q1 first, then q2
 *
 * // Integrate angular velocity
 * Quaternion orientation = Quaternion::identity();
 * Vec3 angular_velocity(0.1, 0.0, 0.0);  // rad/s
 * double dt = 0.01;
 * Quaternion dq = Quaternion::from_axis_angle(
 *     angular_velocity.normalized(),
 *     angular_velocity.magnitude() * dt
 * );
 * orientation = (dq * orientation).normalized();
 * @endcode
 *
 * @see Vec3 for 3D vector operations
 */
class Quaternion {
public:
    double w = 1.0;  ///< Scalar part (cos(θ/2) for rotation quaternions)
    double x = 0.0;  ///< Vector part X component (axis.x * sin(θ/2))
    double y = 0.0;  ///< Vector part Y component (axis.y * sin(θ/2))
    double z = 0.0;  ///< Vector part Z component (axis.z * sin(θ/2))

    // === CONSTRUCTORS ===

    /**
     * @brief Default constructor - creates identity quaternion (no rotation)
     *
     * Initializes to (1, 0, 0, 0) which represents no rotation.
     *
     * @example
     * @code
     * Quaternion q;  // Identity quaternion
     * Vec3 v(1.0, 0.0, 0.0);
     * Vec3 rotated = q.rotate(v);  // Still (1.0, 0.0, 0.0)
     * @endcode
     */
    constexpr Quaternion() = default;

    /**
     * @brief Parameterized constructor
     *
     * Creates a quaternion with specified components.
     * Note: For rotation quaternions, the quaternion must be normalized.
     *
     * @param w Scalar component
     * @param x X component of vector part
     * @param y Y component of vector part
     * @param z Z component of vector part
     */
    constexpr Quaternion(double w, double x, double y, double z)
        : w(w), x(x), y(y), z(z) {}

    /**
     * @brief Create quaternion from axis-angle rotation
     *
     * Creates a rotation quaternion representing a rotation of `angle`
     * radians around the given axis. The axis should be normalized.
     *
     * @param axis Normalized rotation axis
     * @param angle Rotation angle in radians
     * @return Rotation quaternion
     *
     * @example
     * @code
     * // Rotate 45 degrees around the Y axis
     * Quaternion q = Quaternion::from_axis_angle(Vec3::up(), M_PI / 4);
     *
     * // Rotate 90 degrees around arbitrary axis
     * Vec3 axis = Vec3(1.0, 1.0, 1.0).normalized();
     * Quaternion q = Quaternion::from_axis_angle(axis, M_PI / 2);
     * @endcode
     */
    static Quaternion from_axis_angle(const Vec3& axis, double angle) {
        double half_angle = angle * 0.5;
        double s = std::sin(half_angle);
        return Quaternion(
            std::cos(half_angle),
            axis.x * s,
            axis.y * s,
            axis.z * s
        );
    }

    /**
     * @brief Create quaternion from Euler angles
     *
     * Creates a rotation quaternion from pitch, yaw, and roll angles.
     * Uses aerospace convention: Y-P-X (yaw-pitch-roll) intrinsic rotation.
     *
     * @param pitch Rotation around X axis (radians, positive = nose down)
     * @param yaw Rotation around Y axis (radians, positive = nose right)
     * @param roll Rotation around Z axis (radians, positive = right wing down)
     * @return Rotation quaternion
     *
     * @example
     * @code
     * // Level flight, 30 degree right turn
     * Quaternion q = Quaternion::from_euler(0.0, M_PI / 6, 0.0);
     *
     * // 45 degree pitch up, level heading and wings
     * Quaternion q = Quaternion::from_euler(-M_PI / 4, 0.0, 0.0);
     * @endcode
     */
    static Quaternion from_euler(double pitch, double yaw, double roll) {
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        return Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }

    /**
     * @brief Create identity quaternion (no rotation)
     *
     * Returns the identity quaternion (1, 0, 0, 0).
     *
     * @return Identity quaternion
     *
     * @example
     * @code
     * Quaternion q = Quaternion::identity();
     * @endcode
     */
    static Quaternion identity() {
        return Quaternion(1.0, 0.0, 0.0, 0.0);
    }

    // === BASIC OPERATIONS ===

    /**
     * @brief Quaternion multiplication (composition of rotations)
     *
     * Combines two rotations. The result represents applying `other` first,
     * then `this`. Order matters for quaternions!
     *
     * @param other Quaternion to multiply with
     * @return Combined rotation quaternion
     *
     * @example
     * @code
     * Quaternion pitch_up = Quaternion::from_euler(-0.5, 0.0, 0.0);
     * Quaternion turn_right = Quaternion::from_euler(0.0, 0.3, 0.0);
     * Quaternion combined = turn_right * pitch_up;
     * // Applies pitch_up first, then turn_right
     * @endcode
     */
    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        );
    }

    /**
     * @brief Rotate a vector by this quaternion
     *
     * Applies the rotation represented by this quaternion to a vector.
     * Uses the formula: v' = q * v * q⁻¹
     *
     * @param v Vector to rotate
     * @return Rotated vector
     *
     * @example
     * @code
     * Quaternion q = Quaternion::from_axis_angle(Vec3::up(), M_PI / 2);
     * Vec3 forward = Vec3(0.0, 0.0, 1.0);
     * Vec3 right = q.rotate(forward);  // Results in (1, 0, 0)
     * @endcode
     */
    Vec3 rotate(const Vec3& v) const {
        // Convert vector to pure quaternion
        Quaternion qv(0.0, v.x, v.y, v.z);

        // q * v * q_conjugate
        Quaternion result = (*this) * qv * conjugate();

        return Vec3(result.x, result.y, result.z);
    }

    /**
     * @brief Scalar multiplication
     *
     * Multiplies all components by a scalar value.
     * Used for interpolation and scaling operations.
     *
     * @param scalar Scalar value to multiply by
     * @return Scaled quaternion
     */
    Quaternion operator*(double scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    /**
     * @brief Quaternion addition
     *
     * Adds two quaternions component-wise.
     * Used for interpolation and integration operations.
     *
     * @param other Quaternion to add
     * @return Sum quaternion
     */
    Quaternion operator+(const Quaternion& other) const {
        return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z);
    }

    /**
     * @brief Quaternion subtraction
     *
     * Subtracts two quaternions component-wise.
     * Used for calculating rotation differences.
     *
     * @param other Quaternion to subtract
     * @return Difference quaternion
     */
    Quaternion operator-(const Quaternion& other) const {
        return Quaternion(w - other.w, x - other.x, y - other.y, z - other.z);
    }

    /**
     * @brief Negation
     *
     * Returns the quaternion with all components negated.
     * Note: For rotations, q and -q represent the same rotation.
     *
     * @return Negated quaternion
     */
    Quaternion operator-() const {
        return Quaternion(-w, -x, -y, -z);
    }

    // === QUATERNION PROPERTIES ===

    /**
     * @brief Get magnitude (length)
     *
     * Returns the Euclidean norm: sqrt(w² + x² + y² + z²)
     * For rotation quaternions, this should be 1.0.
     *
     * @return Magnitude of the quaternion
     *
     * @example
     * @code
     * Quaternion q = Quaternion::from_euler(0.5, 0.3, 0.2);
     * double mag = q.magnitude();  // Should be ~1.0
     * @endcode
     */
    double magnitude() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    /**
     * @brief Get squared magnitude
     *
     * Returns the squared magnitude without the sqrt operation.
     * Faster than magnitude() for comparisons.
     *
     * @return Squared magnitude
     */
    double magnitude_squared() const {
        return w * w + x * x + y * y + z * z;
    }

    /**
     * @brief Normalize to unit quaternion
     *
     * Returns a normalized version of this quaternion.
     * Critical for rotation quaternions to prevent numerical drift.
     *
     * @return Normalized quaternion (magnitude = 1.0)
     *
     * @example
     * @code
     * Quaternion q(2.0, 0.0, 0.0, 0.0);
     * Quaternion normalized = q.normalized();  // (1.0, 0.0, 0.0, 0.0)
     * @endcode
     */
    Quaternion normalized() const {
        double mag = magnitude();
        if (mag < 1e-8) {
            return identity();  // Return identity if too small
        }
        double inv = 1.0 / mag;
        return Quaternion(w * inv, x * inv, y * inv, z * inv);
    }

    /**
     * @brief Normalize in-place
     *
     * Normalizes this quaternion, modifying its values directly.
     * Call this periodically after integrating angular velocity.
     *
     * @example
     * @code
     * Quaternion orientation = Quaternion::identity();
     * // After some operations...
     * orientation.normalize();  // Keep as valid rotation
     * @endcode
     */
    void normalize() {
        double mag = magnitude();
        if (mag > 1e-8) {
            double inv = 1.0 / mag;
            w *= inv;
            x *= inv;
            y *= inv;
            z *= inv;
        } else {
            w = 1.0;
            x = y = z = 0.0;
        }
    }

    /**
     * @brief Get conjugate (inverse rotation)
     *
     * Returns the conjugate quaternion: (w, -x, -y, -z)
     * For unit quaternions (rotations), conjugate = inverse.
     *
     * @return Conjugate quaternion
     *
     * @example
     * @code
     * Quaternion q = Quaternion::from_euler(0.5, 0.0, 0.0);
     * Quaternion q_inv = q.conjugate();
     * Quaternion combined = q * q_inv;  // Approximately identity
     * @endcode
     */
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    /**
     * @brief Get inverse (reciprocal)
     *
     * Returns the inverse quaternion: q⁻¹ = q* / |q|²
     * For unit quaternions, this is the same as conjugate().
     *
     * @return Inverse quaternion
     */
    Quaternion inverse() const {
        double mag_sq = magnitude_squared();
        if (mag_sq < 1e-16) {
            return identity();  // Can't invert zero quaternion
        }
        Quaternion conj = conjugate();
        double inv = 1.0 / mag_sq;
        return Quaternion(conj.w * inv, conj.x * inv, conj.y * inv, conj.z * inv);
    }

    /**
     * @brief Dot product (similarity measure)
     *
     * Returns the dot product with another quaternion.
     * - 1.0: Identical quaternions
     * - -1.0: Opposite quaternions (same rotation for unit quaternions)
     * - 0.0: Orthogonal quaternions
     *
     * @param other Quaternion to compare with
     * @return Dot product value in range [-1, 1]
     */
    double dot(const Quaternion& other) const {
        return w * other.w + x * other.x + y * other.y + z * other.z;
    }

    // === INTERPOLATION ===

    /**
     * @brief Spherical linear interpolation
     *
     * Smoothly interpolates between two quaternions.
     * Takes the shortest path on the 4D unit sphere.
     * This is the preferred method for interpolating rotations.
     *
     * @param q1 Start quaternion
     * @param q2 End quaternion
     * @param t Interpolation factor [0, 1]
     * @return Interpolated quaternion
     *
     * @example
     * @code
     * Quaternion q1 = Quaternion::identity();
     * Quaternion q2 = Quaternion::from_euler(0.5, 0.3, 0.1);
     * Quaternion halfway = Quaternion::slerp(q1, q2, 0.5);
     * @endcode
     */
    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, double t) {
        // Clamp t to [0, 1]
        t = std::max(0.0, std::min(1.0, t));

        // Calculate cosine of angle between quaternions
        double dot = q1.dot(q2);

        // Use shortest path
        Quaternion q2_temp = q2;
        if (dot < 0.0) {
            q2_temp = -q2_temp;
            dot = -dot;
        }

        // If quaternions are close, use linear interpolation
        if (dot > 0.9995) {
            return Quaternion(
                q1.w + t * (q2_temp.w - q1.w),
                q1.x + t * (q2_temp.x - q1.x),
                q1.y + t * (q2_temp.y - q1.y),
                q1.z + t * (q2_temp.z - q1.z)
            ).normalized();
        }

        // Calculate spherical interpolation
        double theta_0 = std::acos(dot);
        double theta = theta_0 * t;

        double s0 = std::cos(theta) - dot * std::sin(theta) / std::sin(theta_0);
        double s1 = std::sin(theta) / std::sin(theta_0);

        return Quaternion(
            s0 * q1.w + s1 * q2_temp.w,
            s0 * q1.x + s1 * q2_temp.x,
            s0 * q1.y + s1 * q2_temp.y,
            s0 * q1.z + s1 * q2_temp.z
        ).normalized();
    }

    // === CONVERSIONS ===

    /**
     * @brief Convert to 3x3 rotation matrix
     *
     * Fills a 9-element array with the rotation matrix representation.
     * Uses column-major order for compatibility with OpenGL.
     *
     * Matrix layout:
     * [m0 m3 m6]
     * [m1 m4 m7]
     * [m2 m5 m8]
     *
     * @param matrix Output array of 9 doubles
     */
    void to_rotation_matrix(double matrix[9]) const {
        // Normalize first to ensure valid rotation
        Quaternion q = normalized();

        double ww = q.w * q.w;
        double xx = q.x * q.x;
        double yy = q.y * q.y;
        double zz = q.z * q.z;
        double xy = q.x * q.y;
        double xz = q.x * q.z;
        double yz = q.y * q.z;
        double wx = q.w * q.x;
        double wy = q.w * q.y;
        double wz = q.w * q.z;

        matrix[0] = 1.0 - 2.0 * (yy + zz);  // m0
        matrix[1] = 2.0 * (xy + wz);        // m1
        matrix[2] = 2.0 * (xz - wy);        // m2

        matrix[3] = 2.0 * (xy - wz);        // m3
        matrix[4] = 1.0 - 2.0 * (xx + zz);  // m4
        matrix[5] = 2.0 * (yz + wx);        // m5

        matrix[6] = 2.0 * (xz + wy);        // m6
        matrix[7] = 2.0 * (yz - wx);        // m7
        matrix[8] = 1.0 - 2.0 * (xx + yy);  // m8
    }

    /**
     * @brief Get forward direction vector
     *
     * Returns the transformed Z axis (0, 0, 1) after rotation.
     * Useful for getting the "forward" direction from an orientation.
     *
     * @return Forward direction vector
     *
     * @example
     * @code
     * Quaternion q = Quaternion::from_euler(0.0, M_PI / 4, 0.0);
     * Vec3 forward = q.forward();  // Points in rotated Z direction
     * @endcode
     */
    Vec3 forward() const {
        return rotate(Vec3(0.0, 0.0, 1.0));
    }

    /**
     * @brief Get up direction vector
     *
     * Returns the transformed Y axis (0, 1, 0) after rotation.
     * Useful for getting the "up" direction from an orientation.
     *
     * @return Up direction vector
     */
    Vec3 up() const {
        return rotate(Vec3(0.0, 1.0, 0.0));
    }

    /**
     * @brief Get right direction vector
     *
     * Returns the transformed X axis (1, 0, 0) after rotation.
     * Useful for getting the "right" direction from an orientation.
     *
     * @return Right direction vector
     */
    Vec3 right() const {
        return rotate(Vec3(1.0, 0.0, 0.0));
    }

    // === UTILITIES ===

    /**
     * @brief Check if this is a valid rotation quaternion
     *
     * Returns true if the quaternion has non-zero magnitude.
     *
     * @return true if valid, false if degenerate
     */
    bool is_valid() const {
        return magnitude_squared() > 1e-16;
    }

    /**
     * @brief Check if approximately equal to another quaternion
     *
     * Tests if two quaternions represent the same rotation within tolerance.
     * Accounts for q and -q representing the same rotation.
     *
     * @param other Quaternion to compare with
     * @param tolerance Maximum allowed difference (default 1e-6)
     * @return true if approximately equal
     */
    bool approximately_equals(const Quaternion& other, double tolerance = 1e-6) const {
        double dot_product = std::abs(dot(other));
        return dot_product > (1.0 - tolerance);
    }

    /**
     * @brief Stream output for debugging
     *
     * Outputs quaternion in format "(w, x, y, z)"
     *
     * @param os Output stream
     * @param q Quaternion to output
     * @return Reference to output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        os << "(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
        return os;
    }

    /**
     * @brief Get rotation axis and angle
     *
     * Extracts the rotation axis and angle from this quaternion.
     * Useful for understanding what rotation a quaternion represents.
     *
     * @return Pair of (axis, angle_in_radians)
     *
     * @example
     * @code
     * Quaternion q = Quaternion::from_euler(0.5, 0.3, 0.1);
     * auto [axis, angle] = q.to_axis_angle();
     * std::cout << "Rotate " << angle << " rad around " << axis << std::endl;
     * @endcode
     */
    std::pair<Vec3, double> to_axis_angle() const {
        Quaternion q = normalized();

        double angle = 2.0 * std::acos(q.w);
        double s = std::sqrt(1.0 - q.w * q.w);

        Vec3 axis;
        if (s < 1e-6) {
            // Angle is 0 or 2π, axis is arbitrary
            axis = Vec3(1.0, 0.0, 0.0);
        } else {
            axis = Vec3(q.x / s, q.y / s, q.z / s);
        }

        return {axis, angle};
    }
};

} // namespace ballistx

#endif // BALLISTX_QUATERNION_H
