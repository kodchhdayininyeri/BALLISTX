#include <iostream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "ballistics/state_6dof.h"
#include "utils/quaternion.h"
#include "utils/vec3.h"

using namespace ballistx;

/**
 * @brief 6-DOF State demonstration
 *
 * Shows complete rigid body state management
 */
void demo_basic_state() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           6-DOF STATE VECTOR DEMO                        ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    // Create initial state
    Vec3 position(0.0, 0.0, 0.0);
    Vec3 velocity(100.0, 50.0, 0.0);
    Quaternion orientation = Quaternion::identity();
    Vec3 angular_velocity(0.0, 0.0, 10.0);  // 10 rad/s around Z

    State6DOF state(position, velocity, orientation, angular_velocity);

    std::cout << "=== Initial State ===\n";
    std::cout << state << "\n";

    // Access components
    std::cout << "=== Component Access ===\n";
    std::cout << "Position: (" << state.x() << ", " << state.y() << ", " << state.z() << ")\n";
    std::cout << "Velocity: (" << state.vx() << ", " << state.vy() << ", " << state.vz() << ")\n";
    std::cout << "Speed: " << state.speed() << " m/s\n";
    std::cout << "Quaternion: (" << state.qw() << ", " << state.qx() << ", "
             << state.qy() << ", " << state.qz() << ")\n";
    std::cout << "Angular Velocity: (" << state.wx() << ", " << state.wy() << ", "
             << state.wz() << ") rad/s\n";
    std::cout << "Angular Speed: " << state.angular_speed() << " rad/s ("
             << (state.angular_speed() * 60.0 / (2.0 * M_PI)) << " RPM)\n";
}

void demo_state_operations() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           STATE VECTOR OPERATIONS                       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    State6DOF state1(
        Vec3(0.0, 0.0, 0.0),
        Vec3(10.0, 0.0, 0.0),
        Quaternion::identity(),
        Vec3(0.0, 0.0, 1.0)
    );

    State6DOF state2(
        Vec3(1.0, 2.0, 3.0),
        Vec3(0.0, 10.0, 0.0),
        Quaternion::from_axis_angle(Vec3(0.0, 1.0, 0.0), M_PI / 4.0),
        Vec3(0.0, 1.0, 0.0)
    );

    std::cout << "State 1:\n";
    state1.print_compact(std::cout);
    std::cout << "\n\nState 2:\n";
    state2.print_compact(std::cout);
    std::cout << "\n\n";

    // Addition
    State6DOF sum = state1 + state2;
    std::cout << "State1 + State2:\n";
    sum.print_compact(std::cout);
    std::cout << "\n\n";

    // Scaling
    State6DOF scaled = state1 * 2.0;
    std::cout << "State1 * 2.0:\n";
    scaled.print_compact(std::cout);
    std::cout << "\n\n";

    // Subtraction
    State6DOF diff = state2 - state1;
    std::cout << "State2 - State1:\n";
    diff.print_compact(std::cout);
    std::cout << "\n";
}

void demo_derived_quantities() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           DERIVED QUANTITIES                           ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    // 155mm artillery shell parameters
    double mass = 45.0;  // kg
    Vec3 inertia(0.5, 0.5, 0.8);  // kg⋅m² (approximate)

    State6DOF state(
        Vec3(0.0, 1000.0, 0.0),     // 1km altitude
        Vec3(200.0, 100.0, 50.0),   // velocity
        Quaternion::from_euler(M_PI / 6.0, M_PI / 4.0, M_PI / 3.0),
        Vec3(10.0, 5.0, 20.0)       // angular velocity (rad/s)
    );

    std::cout << "State:\n";
    state.print_compact(std::cout);
    std::cout << "\n\n";

    // Derived quantities
    std::cout << "=== Derived Quantities ===\n";
    std::cout << std::fixed << std::setprecision(2);

    double ke = state.get_kinetic_energy(mass, inertia);
    std::cout << "Kinetic Energy: " << ke << " J (" << (ke / 1e6) << " MJ)\n";

    Vec3 momentum = state.get_momentum(mass);
    std::cout << "Linear Momentum: " << momentum << " kg⋅m/s\n";
    std::cout << "Momentum Magnitude: " << momentum.magnitude() << " kg⋅m/s\n";

    Vec3 ang_momentum = state.get_angular_momentum(inertia);
    std::cout << "Angular Momentum: " << ang_momentum << " kg⋅m²/s\n";
    std::cout << "Angular Momentum Magnitude: " << ang_momentum.magnitude() << " kg⋅m²/s\n";

    // Direction vectors
    std::cout << "\n=== Direction Vectors ===\n";
    std::cout << "Forward: " << state.forward() << "\n";
    std::cout << "Up: " << state.up() << "\n";
    std::cout << "Right: " << state.right() << "\n";

    // Body-relative velocity
    Vec3 body_vel = state.get_body_velocity();
    std::cout << "\n=== Body-Relative Velocity ===\n";
    std::cout << "World Velocity: " << state.get_velocity() << "\n";
    std::cout << "Body Velocity: " << body_vel << "\n";
}

void demo_artillery_shell() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           ARTILLERY SHELL STATE                        ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    // 155mm shell with rifling
    Vec3 position(0.0, 0.0, 0.0);
    Vec3 velocity(800.0 * std::cos(M_PI / 4.0),
                  800.0 * std::sin(M_PI / 4.0),
                  0.0);
    double spin_rpm = 15000.0;  // Typical artillery spin rate

    State6DOF shell_state = State6DOF::artillery_shell(position, velocity, spin_rpm);

    std::cout << "=== 155mm Artillery Shell ===\n";
    std::cout << "Muzzle Velocity: " << velocity.magnitude() << " m/s\n";
    std::cout << "Spin Rate: " << spin_rpm << " RPM\n";
    std::cout << "Launch Angle: 45°\n\n";

    std::cout << "Initial State:\n";
    shell_state.print_compact(std::cout);
    std::cout << "\n\n";

    // Check validity
    std::cout << "=== State Validation ===\n";
    std::cout << "Is Valid: " << (shell_state.is_valid() ? "Yes" : "No") << "\n";

    // Show orientation
    std::cout << "\n=== Orientation ===\n";
    std::cout << "Quaternion: " << shell_state.get_orientation() << "\n";
    std::cout << "Forward (velocity direction): " << shell_state.forward() << "\n";
    std::cout << "Up: " << shell_state.up() << "\n";
    std::cout << "Right: " << shell_state.right() << "\n";

    // Angular velocity magnitude
    double spin_rad_s = shell_state.angular_speed();
    double spin_rpm_actual = spin_rad_s * 60.0 / (2.0 * M_PI);
    std::cout << "\n=== Spin ===\n";
    std::cout << "Angular Velocity: " << shell_state.get_angular_velocity() << " rad/s\n";
    std::cout << "Spin Rate: " << spin_rpm_actual << " RPM\n";
    std::cout << "Centrifugal acceleration (at surface): "
             << (spin_rad_s * spin_rad_s * 0.0775) << " m/s²\n";  // r = 155mm/2
}

void demo_state_update() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           STATE UPDATE (RK4 STEP)                       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    // Initial state
    State6DOF state(
        Vec3(0.0, 0.0, 0.0),
        Vec3(100.0, 50.0, 0.0),
        Quaternion::identity(),
        Vec3(0.0, 0.0, 5.0)
    );

    std::cout << "=== Initial State ===\n";
    state.print_compact(std::cout);
    std::cout << "\n\n";

    // Simulate derivative (for RK4)
    // k1 = derivative(state, t) * dt
    State6DOF derivative(
        Vec3(100.0, 50.0, 0.0),      // velocity = d(position)/dt
        Vec3(0.0, -9.81, 0.0),       // acceleration = d(velocity)/dt
        Quaternion(0.0, 0.0, 0.0, 2.5), // d(quat)/dt = 0.5 * omega * quat
        Vec3(0.0, 0.0, -0.5)         // angular acceleration
    );

    double dt = 0.01;

    // Euler step (simplified)
    State6DOF k1 = derivative * dt;
    State6DOF state_new = state + k1;

    // Normalize quaternion
    state_new.normalize_orientation();

    std::cout << "=== After One Time Step (dt = " << dt << " s) ===\n";
    state_new.print_compact(std::cout);
    std::cout << "\n\n";

    std::cout << "=== Changes ===\n";
    std::cout << "Position change: " << (state_new.get_position() - state.get_position()) << "\n";
    std::cout << "Velocity change: " << (state_new.get_velocity() - state.get_velocity()) << "\n";
    std::cout << "Quaternion normalized: " << state_new.get_orientation().magnitude() << "\n";
    std::cout << "State valid: " << (state_new.is_valid() ? "Yes" : "No") << "\n";
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║                                                        ║\n";
    std::cout << "║              BALLISTX 6-DOF STATE DEMO                 ║\n";
    std::cout << "║                                                        ║\n";
    std::cout << "║   Complete 13-dimensional state vector                ║\n";
    std::cout << "║   [x,y,z, vx,vy,vz, qw,qx,qy,qz, wx,wy,wz]           ║\n";
    std::cout << "║                                                        ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n";

    demo_basic_state();
    demo_state_operations();
    demo_derived_quantities();
    demo_artillery_shell();
    demo_state_update();

    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║           6-DOF STATE DEMO COMPLETE                      ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n";

    return 0;
}
