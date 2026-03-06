#include <iostream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "ballistics/projectile.h"
#include "atmosphere/isa_model.h"
#include "utils/vec3.h"

using namespace ballistx;

/**
 * @brief Simple Euler integration for ballistic trajectory
 *
 * This is the most basic form of trajectory calculation.
 * We'll replace this with Runge-Kutta later.
 */
class SimpleEulerIntegrator {
public:
    struct State {
        Vec3 position;      // meters
        Vec3 velocity;      // m/s
        double time;        // seconds
    };

    static State simulate(const Projectile& projectile,
                         const Vec3& launch_velocity,
                         double dt = 0.01) {
        State state;
        state.position = Vec3(0.0, 0.0, 0.0);
        state.velocity = launch_velocity;
        state.time = 0.0;

        Atmosphere atm(0.0);

        std::cout << "\n=== BALLISTX - Simple Trajectory Test ===\n";
        std::cout << "Mass: " << projectile.get_mass() << " kg\n";
        std::cout << "Diameter: " << projectile.get_diameter() * 1000 << " mm\n";
        std::cout << "Drag Coefficient: " << projectile.get_drag_coefficient() << "\n";
        std::cout << "Launch Velocity: " << launch_velocity.magnitude() << " m/s\n";
        std::cout << "Launch Angle: " << std::atan2(launch_velocity.y,
                  std::sqrt(launch_velocity.x * launch_velocity.x +
                           launch_velocity.z * launch_velocity.z)) * 180.0 / M_PI << " degrees\n\n";

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Time(s) | X(m)    | Y(m)    | Z(m)    | Vel(m/s) | Mach\n";
        std::cout << "--------|---------|---------|---------|----------|------\n";

        int step = 0;
        while (state.position.y >= 0.0) {
            // Print every 100 steps
            if (step % 100 == 0) {
                double speed = state.velocity.magnitude();
                double mach = speed / atm.get_speed_of_sound();
                std::cout << std::setw(7) << state.time << " | "
                         << std::setw(7) << state.position.x << " | "
                         << std::setw(7) << state.position.y << " | "
                         << std::setw(7) << state.position.z << " | "
                         << std::setw(8) << speed << " | "
                         << std::setw(4) << mach << "\n";
            }

            // Update atmospheric conditions based on altitude
            atm.set_altitude(state.position.y);

            // Calculate forces
            Vec3 relative_velocity = state.velocity - atm.get_wind_velocity();
            double speed = relative_velocity.magnitude();

            // Drag force: Fd = 0.5 * rho * v^2 * Cd * A
            double dynamic_pressure = 0.5 * atm.get_density() * speed * speed;
            double drag_force_mag = dynamic_pressure * projectile.get_drag_coefficient()
                                   * projectile.get_area();

            Vec3 drag_force = relative_velocity.normalized() * (-drag_force_mag);

            // Gravity force
            Vec3 gravity_force = Vec3(0.0, -Atmosphere::GRAVITY * projectile.get_mass(), 0.0);

            // Total force
            Vec3 total_force = drag_force + gravity_force;

            // Acceleration: a = F / m
            Vec3 acceleration = total_force / projectile.get_mass();

            // Euler integration
            state.velocity += acceleration * dt;
            state.position += state.velocity * dt;
            state.time += dt;

            step++;

            // Safety check
            if (state.time > 1000.0) {
                std::cout << "Simulation timeout (1000s)\n";
                break;
            }
        }

        // Final impact
        std::cout << "\n=== IMPACT ===\n";
        std::cout << "Impact Time: " << state.time << " s\n";
        std::cout << "Impact Position: X=" << state.position.x
                 << " m, Z=" << state.position.z << " m\n";
        std::cout << "Total Range: " << std::sqrt(state.position.x * state.position.x
                 + state.position.z * state.position.z) << " m\n";
        std::cout << "Impact Velocity: " << state.velocity.magnitude() << " m/s\n";

        return state;
    }
};

int main() {
    std::cout << "BALLISTX - Basit Balistik Simulasyon\n";
    std::cout << "====================================\n\n";

    // 155mm top mermisi
    Projectile shell(45.0,        // 45 kg
                    0.155,        // 155mm diameter
                    0.295);       // Cd

    // Fırlatma parametreleri
    double launch_speed = 800.0;  // m/s
    double launch_angle = 45.0 * M_PI / 180.0;  // 45 degrees
    double azimuth = 0.0;         // North direction

    // Fırlatma vektörü
    Vec3 launch_velocity(
        launch_speed * std::cos(launch_angle) * std::sin(azimuth),  // x
        launch_speed * std::sin(launch_angle),                       // y (up)
        launch_speed * std::cos(launch_angle) * std::cos(azimuth)   // z
    );

    // Simülasyon çalıştır
    SimpleEulerIntegrator::simulate(shell, launch_velocity);

    std::cout << "\nSimulation complete!\n";
    return 0;
}
