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

using namespace ballistx;

/**
 * @brief Mach-dependent drag demonstration
 *
 * Shows how drag coefficient varies with Mach number
 * and its effect on trajectory
 */
void print_drag_curve(const DragModel& model) {
    std::cout << "\n=== Drag Coefficient vs Mach Number ===\n";
    std::cout << "Mach  |   Cd    | Regime\n";
    std::cout << "------|---------|------------------\n";

    for (double mach = 0.0; mach <= 3.0; mach += 0.2) {
        double cd = model.get_drag_coefficient(mach);

        std::string regime;
        if (mach < 0.8) regime = "Subsonic";
        else if (mach < 1.2) regime = "Transonic";
        else regime = "Supersonic";

        std::cout << std::fixed << std::setprecision(1)
                 << std::setw(5) << mach << " | "
                 << std::setprecision(3) << std::setw(7) << cd << " | "
                 << regime << "\n";
    }
}

struct SimulationResult {
    double range;
    double time;
    double impact_velocity;
};

SimulationResult simulate_with_constant_cd(const Projectile& base_shell,
                                          const Vec3& launch_velocity,
                                          double constant_cd) {
    RK4Integrator rk4;
    State state(Vec3(0.0, 0.0, 0.0), launch_velocity);
    Atmosphere atm(0.0);

    // Create shell with constant Cd
    Projectile shell = base_shell;
    shell.set_drag_coefficient(constant_cd);

    double time = 0.0;
    const double dt = 0.01;

    auto derivative = [&](const State& s, double t) -> Vec3 {
        const_cast<Atmosphere&>(atm).set_altitude(s.position.y);

        Vec3 relative_velocity = s.velocity - atm.get_wind_velocity();
        double speed = relative_velocity.magnitude();

        double dynamic_pressure = 0.5 * atm.get_density() * speed * speed;
        double drag_force_mag = dynamic_pressure * shell.get_drag_coefficient()
                               * shell.get_area();
        Vec3 drag_force = relative_velocity.normalized() * (-drag_force_mag);
        Vec3 gravity_force = Vec3(0.0, -Atmosphere::GRAVITY * shell.get_mass(), 0.0);

        return (drag_force + gravity_force) / shell.get_mass();
    };

    while (state.position.y >= 0.0 && time < 1000.0) {
        state = rk4.step(state, time, dt, derivative);
        time += dt;
    }

    double fraction = state.position.y / (state.position.y - state.velocity.y * dt);
    double impact_time = time - fraction * dt;
    Vec3 impact_pos = state.position - state.velocity * (fraction * dt);

    return {std::sqrt(impact_pos.x * impact_pos.x + impact_pos.z * impact_pos.z),
            impact_time, state.velocity.magnitude()};
}

SimulationResult simulate_with_mach_cd(const Projectile& base_shell,
                                       const Vec3& launch_velocity,
                                       const DragModel& drag_model) {
    RK4Integrator rk4;
    State state(Vec3(0.0, 0.0, 0.0), launch_velocity);
    Atmosphere atm(0.0);

    double time = 0.0;
    const double dt = 0.01;

    auto derivative = [&](const State& s, double t) -> Vec3 {
        const_cast<Atmosphere&>(atm).set_altitude(s.position.y);

        Vec3 relative_velocity = s.velocity - atm.get_wind_velocity();
        double speed = relative_velocity.magnitude();
        double speed_of_sound = atm.get_speed_of_sound();
        double mach = speed / speed_of_sound;

        // Get Mach-dependent Cd
        double cd = drag_model.get_drag_coefficient(mach);

        double dynamic_pressure = 0.5 * atm.get_density() * speed * speed;
        double drag_force_mag = dynamic_pressure * cd * base_shell.get_area();
        Vec3 drag_force = relative_velocity.normalized() * (-drag_force_mag);
        Vec3 gravity_force = Vec3(0.0, -Atmosphere::GRAVITY * base_shell.get_mass(), 0.0);

        return (drag_force + gravity_force) / base_shell.get_mass();
    };

    // Track Mach regime changes
    std::cout << "\nMach Number During Flight:\n";
    std::cout << "Time(s) | Mach  | Cd     | Altitude(m)\n";
    std::cout << "--------|-------|--------|------------\n";
    int step = 0;
    double last_mach = 0.0;

    while (state.position.y >= 0.0 && time < 1000.0) {
        state = rk4.step(state, time, dt, derivative);
        time += dt;
        step++;

        // Print regime changes
        double speed = state.velocity.magnitude();
        double mach = speed / atm.get_speed_of_sound();
        double cd = drag_model.get_drag_coefficient(mach);

        if (step % 500 == 0 || (last_mach < 1.0 && mach >= 1.0) || (last_mach >= 1.0 && mach < 1.0)) {
            std::cout << std::fixed << std::setprecision(1)
                     << std::setw(7) << time << " | "
                     << std::setprecision(2) << std::setw(5) << mach << " | "
                     << std::setprecision(3) << std::setw(6) << cd << " | "
                     << std::setprecision(0) << std::setw(10) << state.position.y << "\n";
        }
        last_mach = mach;
    }

    double fraction = state.position.y / (state.position.y - state.velocity.y * dt);
    double impact_time = time - fraction * dt;
    Vec3 impact_pos = state.position - state.velocity * (fraction * dt);

    return {std::sqrt(impact_pos.x * impact_pos.x + impact_pos.z * impact_pos.z),
            impact_time, state.velocity.magnitude()};
}

int main() {
    std::cout << "BALLISTX - Mach-Dependent Drag Demonstration\n";
    std::cout << "==============================================\n";

    // Get drag model
    DragModel drag_model = DragModel::standard_artillery();

    // Print drag curve
    print_drag_curve(drag_model);

    // Projectile parameters
    Projectile shell(45.0, 0.155, 0.295);  // 155mm shell

    // Launch parameters
    double launch_speed = 800.0;  // m/s (Mach 2.33 at sea level)
    double launch_angle = 45.0 * M_PI / 180.0;

    Vec3 launch_velocity(
        launch_speed * std::cos(launch_angle),
        launch_speed * std::sin(launch_angle),
        0.0
    );

    std::cout << "\n=== Simulation Parameters ===\n";
    std::cout << "Projectile: 155mm shell (45kg)\n";
    std::cout << "Launch Speed: " << launch_speed << " m/s (Mach "
             << (launch_speed / 340.0) << ")\n";
    std::cout << "Launch Angle: 45 degrees\n";

    // Test 1: Constant Cd (subsonic value)
    std::cout << "\n=== Test 1: Constant Cd (0.155 - subsonic) ===\n";
    auto result1 = simulate_with_constant_cd(shell, launch_velocity, 0.155);
    std::cout << "Range: " << result1.range << " m\n";
    std::cout << "Time: " << result1.time << " s\n";
    std::cout << "Impact Velocity: " << result1.impact_velocity << " m/s\n";

    // Test 2: Constant Cd (supersonic value)
    std::cout << "\n=== Test 2: Constant Cd (0.300 - supersonic) ===\n";
    auto result2 = simulate_with_constant_cd(shell, launch_velocity, 0.300);
    std::cout << "Range: " << result2.range << " m\n";
    std::cout << "Time: " << result2.time << " s\n";
    std::cout << "Impact Velocity: " << result2.impact_velocity << " m/s\n";

    // Test 3: Mach-dependent Cd
    std::cout << "\n=== Test 3: Mach-Dependent Cd (Realistic) ===\n";
    auto result3 = simulate_with_mach_cd(shell, launch_velocity, drag_model);
    std::cout << "\n=== Final Results ===\n";
    std::cout << "Range: " << result3.range << " m\n";
    std::cout << "Time: " << result3.time << " s\n";
    std::cout << "Impact Velocity: " << result3.impact_velocity << " m/s\n";

    // Comparison
    std::cout << "\n=== COMPARISON ===\n";
    std::cout << "Method                  | Range (m) | Difference\n";
    std::cout << "------------------------|-----------|------------\n";
    std::cout << "Constant Cd (0.155)     | " << std::setw(9) << result1.range << " | baseline\n";
    std::cout << "Constant Cd (0.300)     | " << std::setw(9) << result2.range << " | "
             << std::setw(9) << (result2.range - result1.range) << " m\n";
    std::cout << "Mach-dependent Cd       | " << std::setw(9) << result3.range << " | "
             << std::setw(9) << (result3.range - result1.range) << " m\n";

    std::cout << "\nReal-world 155mm range: 18-24 km\n";
    if (result3.range >= 18000 && result3.range <= 24000) {
        std::cout << "✅ Mach-dependent model within real-world range!\n";
    }

    return 0;
}
