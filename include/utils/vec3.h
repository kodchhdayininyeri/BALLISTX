#ifndef BALLISTX_VEC3_H
#define BALLISTX_VEC3_H

#include <cmath>
#include <iostream>

namespace ballistx {

/**
 * @brief 3D Vector class for physics calculations
 *
 * Core math class for the entire engine. All position, velocity,
 * force, and acceleration calculations use Vec3.
 */
class Vec3 {
public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    // Constructors
    constexpr Vec3() = default;
    constexpr Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    // Copy constructor
    constexpr Vec3(const Vec3&) = default;
    Vec3& operator=(const Vec3&) = default;

    // Arithmetic operators
    constexpr Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    constexpr Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    constexpr Vec3 operator*(double scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    constexpr Vec3 operator/(double scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    // Compound assignment operators
    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vec3& operator/=(double scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    // Unary operators
    constexpr Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    // Comparison
    constexpr bool operator==(const Vec3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    constexpr bool operator!=(const Vec3& other) const {
        return !(*this == other);
    }

    // Vector operations
    double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    double magnitude_squared() const {
        return x * x + y * y + z * z;
    }

    Vec3 normalized() const {
        double mag = magnitude();
        if (mag > 0.0) {
            return *this / mag;
        }
        return Vec3(0.0, 0.0, 0.0);
    }

    void normalize() {
        double mag = magnitude();
        if (mag > 0.0) {
            x /= mag;
            y /= mag;
            z /= mag;
        }
    }

    double dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // Distance
    double distance_to(const Vec3& other) const {
        return (*this - other).magnitude();
    }

    double distance_squared_to(const Vec3& other) const {
        return (*this - other).magnitude_squared();
    }

    // Zero check
    constexpr bool is_zero() const {
        return x == 0.0 && y == 0.0 && z == 0.0;
    }

    // Factory methods
    static Vec3 zero() { return Vec3(0.0, 0.0, 0.0); }
    static Vec3 up() { return Vec3(0.0, 1.0, 0.0); }
    static Vec3 forward() { return Vec3(0.0, 0.0, 1.0); }
    static Vec3 right() { return Vec3(1.0, 0.0, 0.0); }

    // Stream output for debugging
    friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};

// Scalar multiplication (left side)
constexpr Vec3 operator*(double scalar, const Vec3& v) {
    return Vec3(v.x * scalar, v.y * scalar, v.z * scalar);
}

} // namespace ballistx

#endif // BALLISTX_VEC3_H
