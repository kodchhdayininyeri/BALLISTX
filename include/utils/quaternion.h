#ifndef BALLISTX_QUATERNION_H
#define BALLISTX_QUATERNION_H

#include "utils/vec3.h"
#include <cmath>
#include <iostream>

namespace ballistx {

/**
 * @brief Quaternion class for 3D rotations
 *
 * Quaternion: q = w + xi + yj + zk
 *
 * Advantages over Euler angles:
 * - No gimbal lock
 * - Smooth interpolation (slerp)
 * - Efficient composition
 * - Numerically stable
 *
 * Common uses:
 * - projectile orientation
 * - camera rotation
 * - angular velocity integration
 * - aerospace attitude control
 */
class Quaternion {
public:
    double w = 1.0;  // Scalar part (cos(θ/2))
    double x = 0.0;  // Vector part X (axis.x * sin(θ/2))
    double y = 0.0;  // Vector part Y (axis.y * sin(θ/2))
    double z = 0.0;  // Vector part Z (axis.z * sin(θ/2))

    // === CONSTRUCTORS ===

    constexpr Quaternion() = default;

    constexpr Quaternion(double w, double x, double y, double z)
        : w(w), x(x), y(y), z(z) {}

    /**
     * @brief Create quaternion from axis-angle rotation
     *
     * @param axis Normalized rotation axis
     * @param angle Rotation angle in radians
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
     * @brief Create quaternion from Euler angles (pitch, yaw, roll)
     *
     * @param pitch Rotation around X axis (radians)
     * @param yaw Rotation around Y axis (radians)
     * @param roll Rotation around Z axis (radians)
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
     */
    static Quaternion identity() {
        return Quaternion(1.0, 0.0, 0.0, 0.0);
    }

    // === BASIC OPERATIONS ===

    /**
     * @brief Quaternion multiplication (composition of rotations)
     *
     * Order matters! q1 * q2 applies q1 first, then q2
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
     * v' = q * v * q⁻¹
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
     */
    Quaternion operator*(double scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    /**
     * @brief Quaternion addition
     */
    Quaternion operator+(const Quaternion& other) const {
        return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z);
    }

    /**
     * @brief Quaternion subtraction
     */
    Quaternion operator-(const Quaternion& other) const {
        return Quaternion(w - other.w, x - other.x, y - other.y, z - other.z);
    }

    /**
     * @brief Negation
     */
    Quaternion operator-() const {
        return Quaternion(-w, -x, -y, -z);
    }

    // === QUATERNION PROPERTIES ===

    /**
     * @brief Get magnitude (length)
     */
    double magnitude() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    /**
     * @brief Get squared magnitude (faster, no sqrt)
     */
    double magnitude_squared() const {
        return w * w + x * x + y * y + z * z;
    }

    /**
     * @brief Normalize to unit quaternion
     *
     * Critical for rotation quaternions to prevent drift
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
     * @brief Get conjugate (for inverse rotation)
     *
     * For unit quaternions: conjugate = inverse
     */
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    /**
     * @brief Get inverse (reciprocal)
     *
     * q⁻¹ = q* / |q|²
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
     * Returns 1.0 for identical quaternions
     * Returns -1.0 for opposite quaternions (same rotation)
     */
    double dot(const Quaternion& other) const {
        return w * other.w + x * other.x + y * other.y + z * other.z;
    }

    // === INTERPOLATION ===

    /**
     * @brief Spherical linear interpolation
     *
     * Smooth interpolation between rotations.
     * Takes the shortest path on the 4D sphere.
     *
     * @param q1 Start quaternion
     * @param q2 End quaternion
     * @param t Interpolation factor [0, 1]
     * @return Interpolated quaternion
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
     * Returns column-major order:
     * [m0 m3 m6]
     * [m1 m4 m7]
     * [m2 m5 m8]
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
     * Returns the transformed Z axis
     */
    Vec3 forward() const {
        return rotate(Vec3(0.0, 0.0, 1.0));
    }

    /**
     * @brief Get up direction vector
     *
     * Returns the transformed Y axis
     */
    Vec3 up() const {
        return rotate(Vec3(0.0, 1.0, 0.0));
    }

    /**
     * @brief Get right direction vector
     *
     * Returns the transformed X axis
     */
    Vec3 right() const {
        return rotate(Vec3(1.0, 0.0, 0.0));
    }

    // === UTILITIES ===

    /**
     * @brief Check if this is a valid rotation quaternion
     */
    bool is_valid() const {
        return magnitude_squared() > 1e-16;
    }

    /**
     * @brief Check if approximately equal to another quaternion
     */
    bool approximately_equals(const Quaternion& other, double tolerance = 1e-6) const {
        double dot_product = std::abs(dot(other));
        return dot_product > (1.0 - tolerance);
    }

    /**
     * @brief Stream output for debugging
     */
    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        os << "(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
        return os;
    }

    /**
     * @brief Get rotation axis and angle
     *
     * @return Pair of (axis, angle_in_radians)
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
