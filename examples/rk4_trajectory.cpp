#include <iostream>
#include <iomanip>
#include <cmath>
#include <functional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "ballistics/projectile.h"
#include "atmosphere/isa_model.h"
#include "utils/vec3.h"
#include "utils/integrator.h"

using namespace ballistx;

/**
 * @brief RK4 Trajectory Simulation
 *
 * Compares Euler vs RK4 integration accuracy
 */
class RK4TrajectorySimulator {
public:
    struct SimulationResult {
        double impact_time;
        double range;
        Vec3 impact_position;
        Vec3 impact_velocity;
        int steps;
    };

    static SimulationResult simulate_with_rk4(const Projectile& projectile,
                                             const Vec3& launch_velocity,
                                             double dt = 0.01) {
        RK4Integrator rk4;
        State state(Vec3(0.0, 0.0, 0.0), launch_velocity);
        Atmosphere atm(0.0);

        double time = 0.0;
        int steps = 0;

        // Derivative function for RK4
        auto derivative = [&](const State& s, double t) -> Vec3 {
            // Update atmosphere based on altitude
            const_cast<Atmosphere&>(atm).set_altitude(s.position.y);

            // Calculate forces
            Vec3 relative_velocity = s.velocity - atm.get_wind_velocity();
            double speed = relative_velocity.magnitude();

            // Drag force
            double dynamic_pressure = 0.5 * atm.get_density() * speed * speed;
            double drag_force_mag = dynamic_pressure * projectile.get_drag_coefficient()
                                   * projectile.get_area();
            Vec3 drag_force = relative_velocity.normalized() * (-drag_force_mag);

            // Gravity force
            Vec3 gravity_force = Vec3(0.0, -Atmosphere::GRAVITY * projectile.get_mass(), 0.0);

            // Total force and acceleration
            Vec3 total_force = drag_force + gravity_force;
            return total_force / projectile.get_mass();
        };

        std::cout << "\n=== RK4 Integration ===\n";
        std::cout << "Time(s) | X(m)    | Y(m)    | Z(m)    | Vel(m/s)\n";
        std::cout << "--------|---------|---------|---------|----------\n";

        while (state.position.y >= 0.0) {
            if (steps % 100 == 0) {
                std::cout << std::fixed << std::setprecision(2)
                         << std::setw(7) << time << " | "
                         << std::setw(7) << state.position.x << " | "
                         << std::setw(7) << state.position.y << " | "
                         << std::setw(7) << state.position.z << " | "
                         << std::setw(8) << state.velocity.magnitude() << "\n";
            }

            state = rk4.step(state, time, dt, derivative);
            time += dt;
            steps++;

            if (time > 1000.0) break;
        }

        // Interpolate exact impact
        double fraction = state.position.y / (state.position.y - state.velocity.y * dt);
        double impact_time = time - fraction * dt;
        Vec3 impact_pos = state.position - state.velocity * (fraction * dt);

        SimulationResult result;
        result.impact_time = impact_time;
        result.range = std::sqrt(impact_pos.x * impact_pos.x + impact_pos.z * impact_pos.z);
        result.impact_position = impact_pos;
        result.impact_velocity = state.velocity;
        result.steps = steps;

        return result;
    }

    static SimulationResult simulate_with_euler(const Projectile& projectile,
                                               const Vec3& launch_velocity,
                                               double dt = 0.01) {
        State state(Vec3(0.0, 0.0, 0.0), launch_velocity);
        Atmosphere atm(0.0);

        double time = 0.0;
        int steps = 0;

        std::cout << "\n=== Euler Integration ===\n";
        std::cout << "Time(s) | X(m)    | Y(m)    | Z(m)    | Vel(m/s)\n";
        std::cout << "--------|---------|---------|---------|----------\n";

        while (state.position.y >= 0.0) {
            if (steps % 100 == 0) {
                std::cout << std::fixed << std::setprecision(2)
                         << std::setw(7) << time << " | "
                         << std::setw(7) << state.position.x << " | "
                         << std::setw(7) << state.position.y << " | "
                         << std::setw(7) << state.position.z << " | "
                         << std::setw(8) << state.velocity.magnitude() << "\n";
            }

            // Update atmosphere
            atm.set_altitude(state.position.y);

            // Calculate forces
            Vec3 relative_velocity = state.velocity - atm.get_wind_velocity();
            double speed = relative_velocity.magnitude();

            double dynamic_pressure = 0.5 * atm.get_density() * speed * speed;
            double drag_force_mag = dynamic_pressure * projectile.get_drag_coefficient()
                                   * projectile.get_area();
            Vec3 drag_force = relative_velocity.normalized() * (-drag_force_mag);
            Vec3 gravity_force = Vec3(0.0, -Atmosphere::GRAVITY * projectile.get_mass(), 0.0);
            Vec3 total_force = drag_force + gravity_force;
            Vec3 acceleration = total_force / projectile.get_mass();

            // Euler step
            state.velocity += acceleration * dt;
            state.position += state.velocity * dt;
            time += dt;
            steps++;

            if (time > 1000.0) break;
        }

        // Interpolate exact impact
        double fraction = state.position.y / (state.position.y - state.velocity.y * dt);
        double impact_time = time - fraction * dt;
        Vec3 impact_pos = state.position - state.velocity * (fraction * dt);

        SimulationResult result;
        result.impact_time = impact_time;
        result.range = std::sqrt(impact_pos.x * impact_pos.x + impact_pos.z * impact_pos.z);
        result.impact_position = impact_pos;
        result.impact_velocity = state.velocity;
        result.steps = steps;

        return result;
    }
};

int main() {
    std::cout << "BALLISTX - Euler vs RK4 Comparison\n";
    std::cout << "====================================\n\n";

    // 155mm top mermisi
    Projectile shell(45.0, 0.155, 0.295);

    // Fırlatma parametreleri
    double launch_speed = 800.0;
    double launch_angle = 45.0 * M_PI / 180.0;

    Vec3 launch_velocity(
        launch_speed * std::cos(launch_angle),
        launch_speed * std::sin(launch_angle),
        0.0
    );

    // Euler
    auto euler_result = RK4TrajectorySimulator::simulate_with_euler(shell, launch_velocity);

    std::cout << "\n=== Euler Impact ===\n";
    std::cout << "Impact Time: " << euler_result.impact_time << " s\n";
    std::cout << "Range: " << euler_result.range << " m\n";
    std::cout << "Impact Velocity: " << euler_result.impact_velocity.magnitude() << " m/s\n";
    std::cout << "Steps: " << euler_result.steps << "\n";

    // RK4
    auto rk4_result = RK4TrajectorySimulator::simulate_with_rk4(shell, launch_velocity);

    std::cout << "\n=== RK4 Impact ===\n";
    std::cout << "Impact Time: " << rk4_result.impact_time << " s\n";
    std::cout << "Range: " << rk4_result.range << " m\n";
    std::cout << "Impact Velocity: " << rk4_result.impact_velocity.magnitude() << " m/s\n";
    std::cout << "Steps: " << rk4_result.steps << "\n";

    // Comparison
    std::cout << "\n=== COMPARISON ===\n";
    double range_diff = rk4_result.range - euler_result.range;
    double time_diff = rk4_result.impact_time - euler_result.impact_time;
    double vel_diff = rk4_result.impact_velocity.magnitude() - euler_result.impact_velocity.magnitude();

    std::cout << "Range Difference: " << std::abs(range_diff) << " m ("
             << (std::abs(range_diff) / euler_result.range * 100) << "%)\n";
    std::cout << "Time Difference: " << std::abs(time_diff) << " s\n";
    std::cout << "Velocity Difference: " << std::abs(vel_diff) << " m/s\n";

    std::cout << "\nReal-world 155mm range: 18-24 km\n";
    if (rk4_result.range >= 18000 && rk4_result.range <= 24000) {
        std::cout << "✅ RK4 within real-world range!\n";
    } else {
        std::cout << "⚠️  Outside real-world range\n";
    }

    return 0;
}
