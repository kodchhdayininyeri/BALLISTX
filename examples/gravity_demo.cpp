#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

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
 * @brief Demonstrate altitude-dependent gravity effects
 */
void print_gravity_analysis() {
    std::cout << "\n=== Gravity vs Altitude Analysis ===\n";
    std::cout << "Altitude(km) | Gravity (m/s²) | Difference | % Change\n";
    std::cout << "-------------|-----------------|-------------|----------\n";

    for (int km = 0; km <= 50; km += 5) {
        double altitude = km * 1000.0;
        double g = GravityModel::get_gravity(altitude);
        double diff = GravityModel::get_gravity_difference(altitude);
        double pct = (diff / GravityModel::STANDARD_GRAVITY) * 100.0;

        std::cout << std::fixed << std::setprecision(0)
                 << std::setw(11) << km << " | "
                 << std::setprecision(4) << std::setw(15) << g << " | "
                 << std::setprecision(4) << std::setw(11) << diff << " | "
                 << std::setprecision(3) << std::setw(8) << pct << "%\n";
    }

    std::cout << "\nKey Insight:\n";
    std::cout << "- At 10km: " << (GravityModel::get_gravity_difference(10000) /
                GravityModel::STANDARD_GRAVITY * 100) << "% reduction\n";
    std::cout << "- At 50km: " << (GravityModel::get_gravity_difference(50000) /
                GravityModel::STANDARD_GRAVITY * 100) << "% reduction\n";
    std::cout << "- For artillery (max ~20km): <1% change\n";
    std::cout << "- For rockets (>50km): Significant impact\n";
}

struct SimulationResult {
    double range;
    double time;
    double max_altitude;
    double impact_velocity;
};

SimulationResult simulate_with_constant_g(const Projectile& shell,
                                          const Vec3& launch_velocity) {
    RK4Integrator rk4;
    State state(Vec3(0.0, 0.0, 0.0), launch_velocity);
    Atmosphere atm(0.0);
    DragModel drag_model = DragModel::standard_artillery();

    double time = 0.0;
    const double dt = 0.01;
    double max_alt = 0.0;

    auto derivative = [&](const State& s, double t) -> Vec3 {
        const_cast<Atmosphere&>(atm).set_altitude(s.position.y);

        // Track max altitude
        if (s.position.y > max_alt) max_alt = s.position.y;

        Vec3 relative_velocity = s.velocity - atm.get_wind_velocity();
        double speed = relative_velocity.magnitude();
        double mach = speed / atm.get_speed_of_sound();

        double cd = drag_model.get_drag_coefficient(mach);
        double dynamic_pressure = 0.5 * atm.get_density() * speed * speed;
        double drag_force_mag = dynamic_pressure * cd * shell.get_area();
        Vec3 drag_force = relative_velocity.normalized() * (-drag_force_mag);

        // CONSTANT gravity
        Vec3 gravity_force(0.0, -GravityModel::STANDARD_GRAVITY * shell.get_mass(), 0.0);

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
            impact_time, max_alt, state.velocity.magnitude()};
}

SimulationResult simulate_with_variable_g(const Projectile& shell,
                                          const Vec3& launch_velocity) {
    RK4Integrator rk4;
    State state(Vec3(0.0, 0.0, 0.0), launch_velocity);
    Atmosphere atm(0.0);
    DragModel drag_model = DragModel::standard_artillery();

    double time = 0.0;
    const double dt = 0.01;
    double max_alt = 0.0;

    auto derivative = [&](const State& s, double t) -> Vec3 {
        const_cast<Atmosphere&>(atm).set_altitude(s.position.y);

        // Track max altitude
        if (s.position.y > max_alt) max_alt = s.position.y;

        Vec3 relative_velocity = s.velocity - atm.get_wind_velocity();
        double speed = relative_velocity.magnitude();
        double mach = speed / atm.get_speed_of_sound();

        double cd = drag_model.get_drag_coefficient(mach);
        double dynamic_pressure = 0.5 * atm.get_density() * speed * speed;
        double drag_force_mag = dynamic_pressure * cd * shell.get_area();
        Vec3 drag_force = relative_velocity.normalized() * (-drag_force_mag);

        // VARIABLE gravity (altitude-dependent)
        double g = GravityModel::get_gravity(s.position.y);
        Vec3 gravity_force(0.0, -g * shell.get_mass(), 0.0);

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
            impact_time, max_alt, state.velocity.magnitude()};
}

int main() {
    std::cout << "BALLISTX - Altitude-Dependent Gravity Model\n";
    std::cout << "=============================================\n";

    // Print gravity analysis
    print_gravity_analysis();

    // Simulation parameters
    std::cout << "\n=== Simulation Parameters ===\n";
    std::cout << "Projectile: 155mm shell (45kg)\n";
    std::cout << "Launch Speed: 800 m/s\n";
    std::cout << "Launch Angle: 45 degrees\n";

    Projectile shell(45.0, 0.155, 0.295);
    double launch_speed = 800.0;
    double launch_angle = 45.0 * M_PI / 180.0;

    Vec3 launch_velocity(
        launch_speed * std::cos(launch_angle),
        launch_speed * std::sin(launch_angle),
        0.0
    );

    // Test 1: Constant gravity
    std::cout << "\n=== Test 1: Constant Gravity (9.807 m/s²) ===\n";
    auto result1 = simulate_with_constant_g(shell, launch_velocity);
    std::cout << "Range: " << result1.range << " m\n";
    std::cout << "Time: " << result1.time << " s\n";
    std::cout << "Max Altitude: " << result1.max_altitude << " m\n";
    std::cout << "Impact Velocity: " << result1.impact_velocity << " m/s\n";

    // Test 2: Variable gravity
    std::cout << "\n=== Test 2: Variable Gravity (Altitude-Dependent) ===\n";
    auto result2 = simulate_with_variable_g(shell, launch_velocity);
    std::cout << "Range: " << result2.range << " m\n";
    std::cout << "Time: " << result2.time << " s\n";
    std::cout << "Max Altitude: " << result2.max_altitude << " m\n";
    std::cout << "Impact Velocity: " << result2.impact_velocity << " m/s\n";

    // Comparison
    std::cout << "\n=== COMPARISON ===\n";
    double range_diff = result2.range - result1.range;
    double time_diff = result2.time - result1.time;
    double alt_diff = result2.max_altitude - result1.max_altitude;
    double vel_diff = result2.impact_velocity - result1.impact_velocity;

    std::cout << "Range Difference: " << range_diff << " m ("
             << (std::abs(range_diff) / result1.range * 100) << "%)\n";
    std::cout << "Time Difference: " << time_diff << " s\n";
    std::cout << "Max Altitude Difference: " << alt_diff << " m\n";
    std::cout << "Impact Velocity Difference: " << vel_diff << " m/s\n";

    std::cout << "\n=== Analysis ===\n";
    std::cout << "At max altitude (" << result2.max_altitude << " m):\n";
    double g_at_max = GravityModel::get_gravity(result2.max_altitude);
    double g_reduction = (GravityModel::STANDARD_GRAVITY - g_at_max) /
                        GravityModel::STANDARD_GRAVITY * 100;
    std::cout << "Gravity: " << g_at_max << " m/s² ("
             << g_reduction << "% reduction)\n";

    std::cout << "\nConclusion:\n";
    if (std::abs(range_diff) < 10) {
        std::cout << "For artillery (< 10km altitude): Gravity variation is negligible\n";
    } else if (std::abs(range_diff) < 100) {
        std::cout << "For artillery: Small but measurable effect\n";
    } else {
        std::cout << "Significant effect - altitude-dependent gravity matters!\n";
    }
    std::cout << "For rockets (> 50km): Altitude-dependent gravity is ESSENTIAL\n";

    return 0;
}
