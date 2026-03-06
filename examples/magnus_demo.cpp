#include <iostream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "aerodynamics/magnus_effect.h"
#include "utils/vec3.h"

using namespace ballistx;
using namespace std;

/**
 * @brief Simplified Magnus effect demonstration
 */
int main() {
    std::cout << "=======================================================\n";
    std::cout << "       MAGNUS EFFECT DEMONSTRATION                   \n";
    std::cout << "=======================================================\n\n";

    std::cout << "== Magnus Effect Explained ==\n";
    std::cout << "When a spinning projectile moves through air:\n";
    std::cout << "1. Air is dragged around by the spinning surface\n";
    std::cout << "2. Velocity difference creates pressure difference\n";
    std::cout << "3. Result: Lateral force perpendicular to spin and velocity\n\n";

    std::cout << "Formula: F_magnus = S * (omega x v)\n\n";

    // Test parameters
    double mass = 45.0;         // kg (155mm shell)
    double diameter = 0.155;    // m
    double radius = diameter * 0.5;
    double rho = 1.225;          // kg/m^3 (sea level)
    double spin_rpm = 15000.0;
    double omega = spin_rpm * 2.0 * M_PI / 60.0;  // rad/s

    Vec3 velocity(800.0 * std::cos(M_PI / 4.0),
                 800.0 * std::sin(M_PI / 4.0),
                 0.0);

    // Spin around velocity vector (backspin for stability)
    Vec3 spin_axis = velocity.normalized();
    Vec3 angular_velocity = spin_axis * omega;

    std::cout << "== Test Scenario ==\n";
    std::cout << "Projectile: 155mm artillery shell\n";
    std::cout << "Mass: " << mass << " kg\n";
    std::cout << "Diameter: " << (diameter * 1000) << " mm\n";
    std::cout << "Velocity: " << velocity.magnitude() << " m/s\n";
    std::cout << "Launch Angle: 45 degrees\n";
    std::cout << "Spin Rate: " << spin_rpm << " RPM\n\n";

    // Calculate Magnus force
    Vec3 magnus_force = MagnusEffect::calculate_force_with_radius(
        velocity, angular_velocity, rho, radius, 0.3
    );

    std::cout << fixed << setprecision(2);
    std::cout << "== Magnus Force Calculation ==\n";
    std::cout << "Velocity: " << velocity << " m/s\n";
    std::cout << "Angular Velocity: " << angular_velocity << " rad/s\n";
    std::cout << "Air Density: " << rho << " kg/m^3\n";
    std::cout << "\nMagnus Force: " << magnus_force << " N\n";

    double magnus_accel = magnus_force.magnitude() / mass;
    std::cout << "Magnus Acceleration: " << magnus_accel << " m/s^2 ("
             << (magnus_accel / 9.81) << " g)\n\n";

    // Gyroscopic stability
    std::cout << "== Gyroscopic Stability ==\n";
    double stability = MagnusEffect::calculate_gyroscopic_stability(
        spin_rpm, velocity.magnitude(), diameter, diameter * 5.0, mass
    );

    std::cout << setprecision(3);
    std::cout << "Stability Factor: " << stability << "\n";
    if (stability > 1.3) {
        std::cout << ">> GYROSCOPICALLY STABLE (S > 1.3)\n";
    } else {
        std::cout << ">> WARNING: UNSTABLE - May tumble\n";
    }
    std::cout << "\n";

    // Spin decay
    std::cout << "== Spin Decay Over Flight ==\n";
    std::cout << setprecision(1);
    std::cout << "Initial Spin: " << spin_rpm << " RPM\n";
    for (int t = 0; t <= 90; t += 15) {
        double current_spin = MagnusEffect::calculate_spin_decay(
            spin_rpm, t, rho, radius
        );
        double decay_pct = (spin_rpm - current_spin) / spin_rpm * 100.0;
        cout << "At t=" << setw(2) << t << "s: " << setw(6) << current_spin << " RPM ("
             << setw(5) << decay_pct << "% decay)\n";
    }
    std::cout << "\n";

    // Lateral deflection estimation
    std::cout << "== Estimated Lateral Deflection ==\n";
    double deflection = MagnusEffect::estimate_lateral_deflection(
        velocity.magnitude(), spin_rpm, 80.0, mass, rho, radius
    );

    cout << setprecision(2);
    cout << "Estimated Lateral Deflection: " << deflection << " m ("
         << (deflection * 3.28084) << " ft)\n\n";

    // Force direction explanation
    std::cout << "== Force Direction ==\n";
    Vec3 force_dir = MagnusEffect::get_force_direction(spin_axis, velocity);
    cout << "Spin Axis: " << spin_axis << "\n";
    cout << "Velocity: " << velocity.normalized() << "\n";
    cout << "Magnus Force Direction: " << force_dir << "\n\n";

    // Sniper application
    std::cout << "== Sniper Application ==\n";
    cout << "For long-range sniping (1000m+):\n";
    cout << "- Magnus effect causes several meters of deflection\n";
    cout << "- Right-hand twist rifling: bullet drifts RIGHT\n";
    cout << "- Must compensate for spin drift\n";
    cout << "- Compensation learned through practice/ballistic tables\n\n";

    // Baseball comparison
    std::cout << "== Baseball Curveball Comparison ==\n";
    double baseball_mass = 0.145;  // kg
    double baseball_radius = 0.037;  // m (74mm diameter / 2)
    double baseball_spin = 2000.0;   // RPM
    Vec3 baseball_vel(40.0, 0.0, 0.0);  // 40 m/s (~90 mph)
    Vec3 baseball_ang_vel(0.0, 0.0, baseball_spin * 2.0 * M_PI / 60.0);

    Vec3 baseball_magnus = MagnusEffect::calculate_force_with_radius(
        baseball_vel, baseball_ang_vel, rho, baseball_radius, 0.5
    );

    double baseball_accel = baseball_magnus.magnitude() / baseball_mass;

    cout << setprecision(2);
    cout << "Baseball Curveball:\n";
    cout << "Velocity: 90 mph (40 m/s)\n";
    cout << "Spin: " << baseball_spin << " RPM\n";
    cout << "Magnus Force: " << baseball_magnus.magnitude() << " N\n";
    cout << "Lateral Acceleration: " << baseball_accel << " m/s^2 ("
         << (baseball_accel / 9.81) << " g)\n";
    cout << "Deflection over 18m (60ft): ";
    double time_to_plate = 18.0 / 40.0;
    double curve_deflection = 0.5 * baseball_accel * time_to_plate * time_to_plate;
    cout << (curve_deflection * 100) << " cm (" << (curve_deflection * 39.37) << " inches)\n\n";

    cout << "=======================================================\n";
    cout << "        MAGNUS DEMO COMPLETE                         \n";
    cout << "=======================================================\n";

    return 0;
}
