#ifndef BALLISTX_VEC3_H
#define BALLISTX_VEC3_H

#include <cmath>
#include <iostream>

namespace ballistx {

/**
 * @brief 3D vector mathematics class for physics calculations
 *
 * Provides comprehensive vector operations for ballistics and guidance calculations
 * including position, velocity, acceleration, force, and angular velocity representations.
 *
 * This is the core math class for the entire ballistx engine. All spatial calculations
 * use Vec3 for representing 3D quantities with full operator support for intuitive
 * mathematical expressions.
 *
 * @example
 * @code
 * Vec3 position(1000.0, 5000.0, 0.0);  // Initial position [m]
 * Vec3 velocity(250.0, 0.0, 0.0);      // Initial velocity [m/s]
 * Vec3 acceleration(0.0, -9.81, 0.0);  // Gravity [m/s²]
 *
 * // Update position using Euler integration
 * position = position + velocity * 0.01;  // dt = 0.01s
 * velocity = velocity + acceleration * 0.01;
 *
 * // Calculate direction and distance
 * Vec3 direction = velocity.normalized();
 * double speed = velocity.magnitude();
 * double distance = position.distance_to(Vec3::zero());
 * @endcode
 *
 * @see Quaternion for 3D rotations
 */
class Vec3 {
public:
    double x = 0.0;  ///< X component (typically East/West or Right/Left)
    double y = 0.0;  ///< Y component (typically Up/Down or altitude)
    double z = 0.0;  ///< Z component (typically North/South or Forward/Back)

    /**
     * @brief Default constructor - initializes to zero vector
     *
     * Creates a Vec3 with all components set to 0.0.
     *
     * @example
     * @code
     * Vec3 v;  // v = (0, 0, 0)
     * @endcode
     */
    constexpr Vec3() = default;

    /**
     * @brief Parameterized constructor
     *
     * Creates a Vec3 with specified x, y, z components.
     *
     * @param x X component value
     * @param y Y component value
     * @param z Z component value
     *
     * @example
     * @code
     * Vec3 position(100.0, 200.0, 300.0);
     * Vec3 velocity(10.0, 0.0, 5.0);
     * @endcode
     */
    constexpr Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    /**
     * @brief Copy constructor
     *
     * Creates a copy of an existing Vec3.
     */
    constexpr Vec3(const Vec3&) = default;

    /**
     * @brief Copy assignment operator
     *
     * Assigns the values from another Vec3 to this one.
     *
     * @return Reference to this Vec3 after assignment
     */
    Vec3& operator=(const Vec3&) = default;

    /**
     * @brief Vector addition operator
     *
     * Adds two vectors component-wise.
     *
     * @param other Vector to add
     * @return New vector representing the sum
     *
     * @example
     * @code
     * Vec3 v1(1.0, 2.0, 3.0);
     * Vec3 v2(4.0, 5.0, 6.0);
     * Vec3 result = v1 + v2;  // (5.0, 7.0, 9.0)
     * @endcode
     */
    constexpr Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    /**
     * @brief Vector subtraction operator
     *
     * Subtracts two vectors component-wise.
     *
     * @param other Vector to subtract
     * @return New vector representing the difference
     *
     * @example
     * @code
     * Vec3 target(100.0, 200.0, 300.0);
     * Vec3 current(50.0, 100.0, 150.0);
     * Vec3 offset = target - current;  // (50.0, 100.0, 150.0)
     * @endcode
     */
    constexpr Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    /**
     * @brief Scalar multiplication operator
     *
     * Multiplies all components by a scalar value.
     *
     * @param scalar Scalar value to multiply by
     * @return New scaled vector
     *
     * @example
     * @code
     * Vec3 velocity(10.0, 20.0, 30.0);
     * Vec3 scaled = velocity * 2.5;  // (25.0, 50.0, 75.0)
     * @endcode
     */
    constexpr Vec3 operator*(double scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    /**
     * @brief Scalar division operator
     *
     * Divides all components by a scalar value.
     *
     * @param scalar Scalar value to divide by
     * @return New scaled vector
     *
     * @example
     * @code
     * Vec3 force(100.0, 200.0, 300.0);
     * Vec3 reduced = force / 2.0;  // (50.0, 100.0, 150.0)
     * @endcode
     */
    constexpr Vec3 operator/(double scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    /**
     * @brief Addition assignment operator
     *
     * Adds another vector to this one in-place.
     *
     * @param other Vector to add
     * @return Reference to this vector after addition
     *
     * @example
     * @code
     * Vec3 position(0.0, 0.0, 0.0);
     * Vec3 displacement(10.0, 20.0, 30.0);
     * position += displacement;  // position is now (10.0, 20.0, 30.0)
     * @endcode
     */
    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    /**
     * @brief Subtraction assignment operator
     *
     * Subtracts another vector from this one in-place.
     *
     * @param other Vector to subtract
     * @return Reference to this vector after subtraction
     */
    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    /**
     * @brief Multiplication assignment operator
     *
     * Multiplies this vector by a scalar in-place.
     *
     * @param scalar Scalar value to multiply by
     * @return Reference to this vector after multiplication
     */
    Vec3& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    /**
     * @brief Division assignment operator
     *
     * Divides this vector by a scalar in-place.
     *
     * @param scalar Scalar value to divide by
     * @return Reference to this vector after division
     */
    Vec3& operator/=(double scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    /**
     * @brief Unary negation operator
     *
     * Returns a vector with all components negated.
     *
     * @return New vector with opposite direction
     *
     * @example
     * @code
     * Vec3 velocity(10.0, -20.0, 30.0);
     * Vec3 opposite = -velocity;  // (-10.0, 20.0, -30.0)
     * @endcode
     */
    constexpr Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    /**
     * @brief Equality comparison operator
     *
     * Checks if two vectors have identical components.
     *
     * @param other Vector to compare with
     * @return true if all components are equal, false otherwise
     */
    constexpr bool operator==(const Vec3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    /**
     * @brief Inequality comparison operator
     *
     * Checks if two vectors differ in any component.
     *
     * @param other Vector to compare with
     * @return true if any component differs, false if identical
     */
    constexpr bool operator!=(const Vec3& other) const {
        return !(*this == other);
    }

    /**
     * @brief Calculate vector magnitude (length)
     *
     * Returns the Euclidean norm of the vector.
     * Uses formula: sqrt(x² + y² + z²)
     *
     * @return Magnitude of the vector
     *
     * @example
     * @code
     * Vec3 v(3.0, 4.0, 0.0);
     * double mag = v.magnitude();  // 5.0
     * @endcode
     */
    double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    /**
     * @brief Calculate squared magnitude
     *
     * Returns the squared length of the vector, avoiding the sqrt operation.
     * Useful for comparisons where the actual magnitude isn't needed.
     *
     * @return Squared magnitude of the vector
     *
     * @example
     * @code
     * Vec3 v(3.0, 4.0, 0.0);
     * double mag_sq = v.magnitude_squared();  // 25.0
     * @endcode
     */
    double magnitude_squared() const {
        return x * x + y * y + z * z;
    }

    /**
     * @brief Return normalized (unit) vector
     *
     * Returns a new vector with same direction but magnitude of 1.0.
     * Returns zero vector if this vector has zero magnitude.
     *
     * @return New normalized vector
     *
     * @example
     * @code
     * Vec3 velocity(100.0, 0.0, 0.0);
     * Vec3 direction = velocity.normalized();  // (1.0, 0.0, 0.0)
     * @endcode
     */
    Vec3 normalized() const {
        double mag = magnitude();
        if (mag > 0.0) {
            return *this / mag;
        }
        return Vec3(0.0, 0.0, 0.0);
    }

    /**
     * @brief Normalize this vector in-place
     *
     * Converts this vector to a unit vector with magnitude 1.0.
     * Does nothing if the vector has zero magnitude.
     *
     * @example
     * @code
     * Vec3 v(3.0, 4.0, 0.0);
     * v.normalize();  // v is now (0.6, 0.8, 0.0)
     * @endcode
     */
    void normalize() {
        double mag = magnitude();
        if (mag > 0.0) {
            x /= mag;
            y /= mag;
            z /= mag;
        }
    }

    /**
     * @brief Calculate dot product with another vector
     *
     * Returns the scalar dot product: x₁x₂ + y₁y₂ + z₁z₂
     * Related to the angle between vectors: a·b = |a||b|cos(θ)
     *
     * @param other Second vector
     * @return Scalar dot product value
     *
     * @example
     * @code
     * Vec3 v1(1.0, 0.0, 0.0);
     * Vec3 v2(0.0, 1.0, 0.0);
     * double dot = v1.dot(v2);  // 0.0 (orthogonal)
     *
     * Vec3 v3(1.0, 0.0, 0.0);
     * Vec3 v4(1.0, 0.0, 0.0);
     * double dot2 = v3.dot(v4);  // 1.0 (parallel, unit vectors)
     * @endcode
     */
    double dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    /**
     * @brief Calculate cross product with another vector
     *
     * Returns a vector perpendicular to both input vectors.
     * The result follows the right-hand rule.
     *
     * @param other Second vector
     * @return New vector representing the cross product
     *
     * @example
     * @code
     * Vec3 x_axis(1.0, 0.0, 0.0);
     * Vec3 y_axis(0.0, 1.0, 0.0);
     * Vec3 z_axis = x_axis.cross(y_axis);  // (0.0, 0.0, 1.0)
     * @endcode
     */
    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    /**
     * @brief Calculate Euclidean distance to another point
     *
     * Returns the distance between this vector and another.
     *
     * @param other Target position
     * @return Distance between the two vectors
     *
     * @example
     * @code
     * Vec3 missile_pos(0.0, 0.0, 0.0);
     * Vec3 target_pos(100.0, 100.0, 0.0);
     * double range = missile_pos.distance_to(target_pos);  // ~141.42
     * @endcode
     */
    double distance_to(const Vec3& other) const {
        return (*this - other).magnitude();
    }

    /**
     * @brief Calculate squared distance to another point
     *
     * Returns the squared distance, avoiding sqrt operation.
     * Useful for distance comparisons without needing actual distance.
     *
     * @param other Target position
     * @return Squared distance between the two vectors
     */
    double distance_squared_to(const Vec3& other) const {
        return (*this - other).magnitude_squared();
    }

    /**
     * @brief Check if vector is zero
     *
     * Returns true if all components are exactly 0.0.
     *
     * @return true if zero vector, false otherwise
     */
    constexpr bool is_zero() const {
        return x == 0.0 && y == 0.0 && z == 0.0;
    }

    /**
     * @brief Factory method for zero vector
     *
     * @return Vec3(0.0, 0.0, 0.0)
     */
    static Vec3 zero() { return Vec3(0.0, 0.0, 0.0); }

    /**
     * @brief Factory method for up direction
     *
     * @return Unit vector pointing up (0.0, 1.0, 0.0)
     */
    static Vec3 up() { return Vec3(0.0, 1.0, 0.0); }

    /**
     * @brief Factory method for forward direction
     *
     * @return Unit vector pointing forward (0.0, 0.0, 1.0)
     */
    static Vec3 forward() { return Vec3(0.0, 0.0, 1.0); }

    /**
     * @brief Factory method for right direction
     *
     * @return Unit vector pointing right (1.0, 0.0, 0.0)
     */
    static Vec3 right() { return Vec3(1.0, 0.0, 0.0); }

    /**
     * @brief Array access operator (const)
     *
     * Allows indexed access to vector components: 0=x, 1=y, 2=z
     *
     * @param index Component index (0-2)
     * @return Component value
     *
     * @example
     * @code
     * Vec3 v(1.0, 2.0, 3.0);
     * double x = v[0];  // 1.0
     * double y = v[1];  // 2.0
     * double z = v[2];  // 3.0
     * @endcode
     */
    double operator[](size_t index) const {
        if (index == 0) return x;
        if (index == 1) return y;
        return z;
    }

    /**
     * @brief Array access operator (non-const)
     *
     * Allows indexed access and modification of vector components
     *
     * @param index Component index (0-2)
     * @return Reference to component
     */
    double& operator[](size_t index) {
        if (index == 0) return x;
        if (index == 1) return y;
        return z;
    }

    /**
     * @brief Stream output operator for debugging
     *
     * Outputs the vector in format "(x, y, z)"
     *
     * @param os Output stream
     * @param v Vector to output
     * @return Reference to the output stream
     *
     * @example
     * @code
     * Vec3 v(1.0, 2.0, 3.0);
     * std::cout << v;  // Outputs: (1, 2, 3)
     * @endcode
     */
    friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};

/**
 * @brief Scalar multiplication (left-side operator)
 *
 * Allows scalar multiplication in the form: scalar * vector
 *
 * @param scalar Scalar value
 * @param v Vector to multiply
 * @return New scaled vector
 *
 * @example
 * @code
 * Vec3 v(1.0, 2.0, 3.0);
 * Vec3 result = 2.5 * v;  // (2.5, 5.0, 7.5)
 * @endcode
 */
constexpr Vec3 operator*(double scalar, const Vec3& v) {
    return Vec3(v.x * scalar, v.y * scalar, v.z * scalar);
}

} // namespace ballistx

#endif // BALLISTX_VEC3_H
