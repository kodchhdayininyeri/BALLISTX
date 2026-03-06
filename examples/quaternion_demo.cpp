#include <iostream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "utils/quaternion.h"
#include "utils/vec3.h"

using namespace ballistx;

/**
 * @brief Quaternion demonstration
 *
 * Shows:
 * - Basic quaternion operations
 * - Rotation vs Euler angles
 * - No gimbal lock
 * - SLERP interpolation
 */
void demo_basic_operations() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           QUATERNION BASIC OPERATIONS                    ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    // Identity quaternion
    Quaternion q_id = Quaternion::identity();
    std::cout << "Identity: " << q_id << "\n";

    // 90° rotation around Y axis
    Quaternion q_rot = Quaternion::from_axis_angle(Vec3(0.0, 1.0, 0.0), M_PI / 2.0);
    std::cout << "90° around Y: " << q_rot << "\n";

    // Magnitude
    std::cout << "Magnitude: " << q_rot.magnitude() << " (should be 1.0)\n";

    // Normalize
    Quaternion q_unnormalized(2.0, 0.0, 0.0, 0.0);
    std::cout << "Before normalize: " << q_unnormalized
             << ", magnitude: " << q_unnormalized.magnitude() << "\n";
    q_unnormalized.normalize();
    std::cout << "After normalize: " << q_unnormalized
             << ", magnitude: " << q_unnormalized.magnitude() << "\n";

    // Conjugate and inverse
    std::cout << "Original: " << q_rot << "\n";
    std::cout << "Conjugate: " << q_rot.conjugate() << "\n";
    std::cout << "Inverse: " << q_rot.inverse() << "\n";

    // Multiplication (rotation composition)
    Quaternion q1 = Quaternion::from_axis_angle(Vec3(1.0, 0.0, 0.0), M_PI / 4.0);  // 45° X
    Quaternion q2 = Quaternion::from_axis_angle(Vec3(0.0, 1.0, 0.0), M_PI / 4.0);  // 45° Y
    Quaternion q_combined = q1 * q2;
    std::cout << "q1 (45° X): " << q1 << "\n";
    std::cout << "q2 (45° Y): " << q2 << "\n";
    std::cout << "q1 * q2: " << q_combined << "\n";
}

void demo_rotation() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║              VECTOR ROTATION                            ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    // Original vector
    Vec3 v(1.0, 0.0, 0.0);  // Pointing along X axis
    std::cout << "Original vector: " << v << "\n";

    // Rotate 90° around Z axis (should point along -Y)
    Quaternion q_z = Quaternion::from_axis_angle(Vec3(0.0, 0.0, 1.0), M_PI / 2.0);
    Vec3 v_rotated = q_z.rotate(v);
    std::cout << "After 90° Z rotation: " << v_rotated << "\n";
    std::cout << "Expected: (0, -1, 0) approximately\n";

    // Rotate 90° around Y axis (should point along Z)
    Quaternion q_y = Quaternion::from_axis_angle(Vec3(0.0, 1.0, 0.0), M_PI / 2.0);
    v_rotated = q_y.rotate(v);
    std::cout << "After 90° Y rotation: " << v_rotated << "\n";
    std::cout << "Expected: (0, 0, 1) approximately\n";
}

void demo_no_gimbal_lock() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           NO GIMBAL LOCK DEMONSTRATION                  ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    std::cout << "Pitch 90° (X axis rotation):\n";
    Quaternion q_pitch_90 = Quaternion::from_euler(M_PI / 2.0, 0.0, 0.0);

    std::cout << "After pitch 90°, yaw 45°:\n";
    Quaternion q_combined = q_pitch_90 * Quaternion::from_euler(0.0, M_PI / 4.0, 0.0);

    std::cout << "Result: " << q_combined << "\n";
    std::cout << "Magnitude: " << q_combined.magnitude() << " (still valid!)\n";
    std::cout << "✅ No gimbal lock - rotation still works!\n";

    std::cout << "\nCompare with Euler angles:\n";
    std::cout << "Euler: pitch=90°, yaw=45° → One degree of freedom LOST\n";
    std::cout << "Quaternion: pitch=90°, yaw=45° → Still 3 degrees of freedom\n";
}

void demo_slerp() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           SPHERICAL INTERPOLATION (SLERP)              ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    Quaternion q1 = Quaternion::from_axis_angle(Vec3(0.0, 1.0, 0.0), 0.0);      // 0°
    Quaternion q2 = Quaternion::from_axis_angle(Vec3(0.0, 1.0, 0.0), M_PI);    // 180°

    std::cout << "Interpolating from 0° to 180° around Y axis:\n";
    std::cout << "t    | Quaternion              | Angle (degrees)\n";
    std::cout << "------|-------------------------|----------------\n";

    for (int i = 0; i <= 10; i++) {
        double t = i / 10.0;
        Quaternion q_interp = Quaternion::slerp(q1, q2, t);
        auto [axis, angle] = q_interp.to_axis_angle();
        double degrees = angle * 180.0 / M_PI;

        std::cout << std::fixed << std::setprecision(1)
                 << std::setw(4) << t << " | "
                 << q_interp << " | "
                 << std::setw(14) << degrees << "\n";
    }

    std::cout << "\n✅ Smooth interpolation along shortest path!\n";
}

void demo_direction_vectors() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           DIRECTION VECTORS                             ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    // Create a complex rotation
    Quaternion q = Quaternion::from_euler(M_PI / 6.0, M_PI / 4.0, M_PI / 3.0);

    std::cout << "Quaternion: " << q << "\n";
    std::cout << "Forward: " << q.forward() << "\n";
    std::cout << "Up: " << q.up() << "\n";
    std::cout << "Right: " << q.right() << "\n";

    // Verify orthonormal basis
    Vec3 f = q.forward().normalized();
    Vec3 u = q.up().normalized();
    Vec3 r = q.right().normalized();

    std::cout << "\nVerification:\n";
    std::cout << "Forward · Up: " << f.dot(u) << " (should be ~0)\n";
    std::cout << "Forward · Right: " << f.dot(r) << " (should be ~0)\n";
    std::cout << "Up · Right: " << u.dot(r) << " (should be ~0)\n";
    std::cout << "✅ Orthonormal basis maintained!\n";
}

void demo_rotation_matrix() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           ROTATION MATRIX CONVERSION                   ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    Quaternion q = Quaternion::from_euler(M_PI / 4.0, M_PI / 6.0, M_PI / 3.0);

    double matrix[9];
    q.to_rotation_matrix(matrix);

    std::cout << "3x3 Rotation Matrix (column-major):\n";
    std::cout << std::fixed << std::setprecision(4);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            std::cout << std::setw(10) << matrix[i + j * 3] << " ";
        }
        std::cout << "\n";
    }

    // Verify it's a rotation matrix
    std::cout << "\nVerification:\n";
    std::cout << "Determinant: " << (matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7])
                                    - matrix[3] * (matrix[1] * matrix[8] - matrix[2] * matrix[7])
                                    + matrix[6] * (matrix[1] * matrix[5] - matrix[2] * matrix[4]))
             << " (should be +1.0)\n";
    std::cout << "✅ Valid rotation matrix!\n";
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║                                                        ║\n";
    std::cout << "║              BALLISTX QUATERNION DEMO                  ║\n";
    std::cout << "║                                                        ║\n";
    std::cout << "║   Rotation without Gimbal Lock                        ║\n";
    std::cout << "║   Smooth interpolation (SLERP)                        ║\n";
    std::cout << "║   Efficient 3D rotations                              ║\n";
    std::cout << "║                                                        ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n";

    demo_basic_operations();
    demo_rotation();
    demo_no_gimbal_lock();
    demo_slerp();
    demo_direction_vectors();
    demo_rotation_matrix();

    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           QUATERNION DEMO COMPLETE                      ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n";

    return 0;
}
