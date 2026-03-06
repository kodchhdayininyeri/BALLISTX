#include <iostream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "ballistics/projectile.h"
#include "atmosphere/isa_model.h"
#include "aerodynamics/drag_model.h"
#include "utils/vec3.h"
#include "utils/integrator.h"
#include "utils/gravity_model.h"

using namespace ballistx;

/**
 * @brief BALLISTX Main Demo
 *
 * Simulates a 155mm artillery shell using RK4 integration with:
 * - Altitude-dependent gravity
 * - Mach-dependent drag coefficient
 * - ISA atmosphere model
 * - Runge-Kutta 4th order integration
 */
int main() {
    std::cout << "╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║   BALLISTX - Real-Time Ballistics Simulation Engine    ║\n";
    std::cout << "║                                                        ║\n";
    std::cout << "║   Version: 0.1.0                                      ║\n";
    std::cout << "║   C++ Standard: C++17                                 ║\n";
    std::cout << "║   Integration: Runge-Kutta 4th Order                  ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    // === PROJECTILE PARAMETERS ===
    std::cout << "=== PROJECTILE SPECIFICATIONS ===\n";
    std::cout << "Type: 155mm Artillery Shell (M107)\n";
    std::cout << "Mass: 45.0 kg\n";
    std::cout << "Diameter: 155 mm\n";
    std::cout << "Drag Model: G7 (Mach-dependent)\n\n";

    Projectile shell(45.0,           // 45 kg
                    0.155,           // 155 mm diameter
                    0.295);          // Base drag coefficient

    // === LAUNCH PARAMETERS ===
    std::cout << "=== LAUNCH PARAMETERS ===\n";
    double launch_speed = 900.0;     // m/s
    double launch_angle = 45.0 * M_PI / 180.0;  // 45 degrees
    double azimuth = 0.0;            // North direction

    Vec3 launch_velocity(
        launch_speed * std::cos(launch_angle) * std::sin(azimuth),
        launch_speed * std::sin(launch_angle),
        launch_speed * std::cos(launch_angle) * std::cos(azimuth)
    );

    std::cout << "Muzzle Velocity: " << launch_speed << " m/s (Mach "
             << (launch_speed / 340.0) << ")\n";
    std::cout << "Launch Angle: 45.0°\n";
    std::cout << "Azimuth: 0.0° (North)\n\n";

    // === SIMULATION SETUP ===
    RK4Integrator rk4;
    State state(Vec3(0.0, 0.0, 0.0), launch_velocity);
    Atmosphere atm(0.0);
    DragModel drag_model = DragModel::standard_artillery();

    double time = 0.0;
    const double dt = 0.01;
    int step = 0;
    double max_altitude = 0.0;
    double max_mach = 0.0;

    // === DERIVATIVE FUNCTION ===
    auto derivative = [&](const State& s, double t) -> Vec3 {
        // Update atmosphere based on altitude
        const_cast<Atmosphere&>(atm).set_altitude(s.position.y);

        // Track statistics
        if (s.position.y > max_altitude) max_altitude = s.position.y;

        Vec3 relative_velocity = s.velocity - atm.get_wind_velocity();
        double speed = relative_velocity.magnitude();
        double mach = speed / atm.get_speed_of_sound();

        if (mach > max_mach) max_mach = mach;

        // Mach-dependent drag coefficient
        double cd = drag_model.get_drag_coefficient(mach);

        // Drag force: F = 0.5 * rho * v² * Cd * A
        double dynamic_pressure = 0.5 * atm.get_density() * speed * speed;
        double drag_force_mag = dynamic_pressure * cd * shell.get_area();
        Vec3 drag_force = relative_velocity.normalized() * (-drag_force_mag);

        // Altitude-dependent gravity
        double g = GravityModel::get_gravity(s.position.y);
        Vec3 gravity_force(0.0, -g * shell.get_mass(), 0.0);

        // Total acceleration
        return (drag_force + gravity_force) / shell.get_mass();
    };

    // === TRAJECTORY OUTPUT ===
    std::cout << "=== TRAJECTORY DATA ===\n";
    std::cout << "  Time(s) |   Alt(m) |   Down(m) |   Vel(m/s) |  Mach |     Cd  | Regime\n";
    std::cout << "----------|----------|----------|------------|-------|---------|----------\n";

    std::cout << std::fixed << std::setprecision(2);

    while (state.position.y >= 0.0 && time < 1000.0) {
        // Print every 2 seconds
        if (step % 200 == 0) {
            double speed = state.velocity.magnitude();
            double mach = speed / atm.get_speed_of_sound();
            double cd = drag_model.get_drag_coefficient(mach);

            std::string regime;
            if (mach < 0.8) regime = "Subsonic    ";
            else if (mach < 1.2) regime = "Transonic  ";
            else regime = "Supersonic ";

            double range = std::sqrt(state.position.x * state.position.x +
                                    state.position.z * state.position.z);

            std::cout << std::setw(9) << time << " | "
                     << std::setw(8) << state.position.y << " | "
                     << std::setw(8) << range << " | "
                     << std::setw(10) << speed << " | "
                     << std::setw(5) << mach << " | "
                     << std::setw(7) << cd << " | "
                     << regime << "\n";
        }

        // RK4 integration step
        state = rk4.step(state, time, dt, derivative);
        time += dt;
        step++;
    }

    // === IMPACT CALCULATION ===
    double fraction = state.position.y / (state.position.y - state.velocity.y * dt);
    double impact_time = time - fraction * dt;
    Vec3 impact_pos = state.position - state.velocity * (fraction * dt);
    double impact_range = std::sqrt(impact_pos.x * impact_pos.x +
                                    impact_pos.z * impact_pos.z);

    // === FINAL RESULTS ===
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║                   IMPACT DATA                           ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

    std::cout << "=== TRAJECTORY SUMMARY ===\n";
    std::cout << "Impact Time:      " << std::setprecision(3) << impact_time << " s\n";
    std::cout << "Total Range:      " << std::setprecision(1) << impact_range << " m ("
             << std::setprecision(2) << (impact_range / 1000.0) << " km)\n";
    std::cout << "Max Altitude:     " << std::setprecision(1) << max_altitude << " m\n";
    std::cout << "Impact Velocity:  " << std::setprecision(2) << state.velocity.magnitude() << " m/s\n";
    std::cout << "Max Mach Number:  " << std::setprecision(2) << max_mach << "\n\n";

    std::cout << "=== PHYSICS MODEL USED ===\n";
    std::cout << "Integration:      Runge-Kutta 4th Order (O(dt⁴))\n";
    std::cout << "Atmosphere:       ISA Model (altitude-dependent)\n";
    std::cout << "Gravity:          Altitude-dependent (g = g₀ × (R/(R+h))²)\n";
    std::cout << "Drag:             Mach-dependent Cd (G7 curve)\n";
    std::cout << "Time Step:        " << dt << " s\n";
    std::cout << "Total Steps:      " << step << "\n\n";

    std::cout << "=== ACCURACY VALIDATION ===\n";
    std::cout << "Real-world 155mm range (M107): 18-24 km\n";
    std::cout << "Simulated range:                 " << (impact_range / 1000.0) << " km\n";

    if (impact_range >= 18000 && impact_range <= 24000) {
        std::cout << "✅ WITHIN REAL-WORLD RANGE - Model validated!\n";
    } else {
        double error = ((impact_range - 21000) / 21000.0) * 100.0;
        std::cout << "⚠️  Outside typical range ("
                 << std::abs(error) << "% error from 21km mean)\n";
    }

    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║   BALLISTX Simulation Complete                          ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n";

    return 0;
}
