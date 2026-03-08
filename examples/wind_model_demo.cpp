/**
 * @file wind_model_demo.cpp
 * @brief Demonstration of wind model with turbulence
 *
 * This demo shows:
 * - Constant wind vector setup
 * - Turbulence generation (Gaussian noise)
 * - Relative velocity calculation for drag force
 * - Effect of wind on projectile trajectory
 */

#include "atmosphere/isa_model.h"
#include "utils/vec3.h"
#include <iostream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

void print_separator(const std::string& title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(60, '=') << "\n\n";
}

void print_vec3(const std::string& label, const Vec3& v) {
    std::cout << std::left << std::setw(25) << label
              << std::fixed << std::setprecision(3)
              << "(" << std::setw(8) << v.x << ", "
              << std::setw(8) << v.y << ", "
              << std::setw(8) << v.z << ")\n";
}

int main() {
    print_separator("WIND MODEL WITH TURBULENCE DEMO");

    // =========================================================================
    // Part 1: Constant Wind
    // =========================================================================
    {
        std::cout << "PART 1: CONSTANT WIND VECTOR\n\n";

        Atmosphere atm(1000.0);  // 1000m altitude

        // Set headwind (blowing from front, -X direction)
        atm.set_wind(-15.0, 0.0, 0.0);  // 15 m/s headwind
        print_vec3("Wind velocity:", atm.get_wind_velocity());

        // Set crosswind (blowing from left, +Y direction)
        atm.set_wind(0.0, 8.0, 0.0);  // 8 m/s crosswind
        print_vec3("Crosswind velocity:", atm.get_wind_velocity());

        // Set tailwind with vertical component
        atm.set_wind(10.0, 5.0, -2.0);  // tailwind + crosswind + downdraft
        print_vec3("Complex wind vector:", atm.get_wind_velocity());
    }

    // =========================================================================
    // Part 2: Turbulence Model
    // =========================================================================
    {
        print_separator("PART 2: TURBULENCE MODEL");

        Atmosphere atm(500.0);
        atm.set_wind(20.0, 0.0, 0.0);  // 20 m/s wind

        std::cout << "Base wind: 20 m/s (X direction)\n";
        std::cout << "Turbulence intensity: 15%\n\n";

        atm.set_turbulence_intensity(0.15);  // 15% turbulence
        atm.enable_turbulence(true);

        std::cout << "Turbulent wind samples (10 samples):\n";
        std::cout << std::string(50, '-') << "\n";

        for (int i = 0; i < 10; ++i) {
            Vec3 turb = atm.get_turbulent_wind();
            std::cout << "Sample " << (i + 1) << ": ";
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "(" << std::setw(7) << turb.x << ", "
                      << std::setw(7) << turb.y << ", "
                      << std::setw(7) << turb.z << ") m/s\n";
        }
    }

    // =========================================================================
    // Part 3: Relative Velocity Calculation
    // =========================================================================
    {
        print_separator("PART 3: RELATIVE VELOCITY");

        Atmosphere atm;
        atm.set_wind(10.0, 5.0, 0.0);  // Wind from front-left
        print_vec3("Wind velocity:", atm.get_wind_velocity());

        // Projectile velocity (typical artillery shell)
        Vec3 projectile_velocity(800.0, 0.0, 0.0);  // 800 m/s forward
        print_vec3("Projectile velocity:", projectile_velocity);

        Vec3 relative_vel = atm.get_relative_velocity(projectile_velocity);
        print_vec3("Relative velocity:", relative_vel);

        std::cout << "\nPhysical interpretation:\n";
        std::cout << "  Headwind component increases apparent airspeed\n";
        std::cout << "  Crosswind creates side force (yaw)\n";
    }

    // =========================================================================
    // Part 4: Drag Force with Wind
    // =========================================================================
    {
        print_separator("PART 4: DRAG FORCE WITH WIND");

        Atmosphere atm;
        atm.set_wind(-20.0, 0.0, 0.0);  // 20 m/s headwind (opposing motion)

        double density = atm.get_density();
        std::cout << "Air density: " << std::fixed << std::setprecision(3)
                  << density << " kg/m^3\n";

        double diameter = 0.155;  // 155mm artillery shell
        double area = M_PI * diameter * diameter / 4.0;
        std::cout << "Projectile area: " << std::setprecision(4)
                  << area << " m^2\n";

        double cd = 0.3;  // Drag coefficient
        std::cout << "Drag coefficient: " << cd << "\n\n";

        // No wind case
        Vec3 v_no_wind(800.0, 0.0, 0.0);
        double v_mag_no_wind = v_no_wind.magnitude();
        double drag_no_wind = 0.5 * density * v_mag_no_wind * v_mag_no_wind * cd * area;
        std::cout << "NO WIND CASE:\n";
        std::cout << "  Velocity: " << v_mag_no_wind << " m/s\n";
        std::cout << "  Drag force: " << std::scientific << drag_no_wind << " N\n\n";

        // With headwind case
        Vec3 v_with_wind = atm.get_relative_velocity(v_no_wind);
        double v_mag_with_wind = v_with_wind.magnitude();
        double drag_with_wind = 0.5 * density * v_mag_with_wind * v_mag_with_wind * cd * area;
        std::cout << std::fixed << "WITH 20 m/s HEADWIND:\n";
        std::cout << "  Relative velocity: " << v_mag_with_wind << " m/s\n";
        std::cout << "  Drag force: " << std::scientific << drag_with_wind << " N\n\n";

        double drag_increase = ((drag_with_wind - drag_no_wind) / drag_no_wind) * 100.0;
        std::cout << std::fixed << "Drag increase: " << std::setprecision(1)
                  << drag_increase << "%\n";
    }

    // =========================================================================
    // Part 5: Turbulence Statistics
    // =========================================================================
    {
        print_separator("PART 5: TURBULENCE STATISTICS");

        Atmosphere atm;
        atm.set_wind(15.0, 0.0, 0.0);
        atm.set_turbulence_intensity(0.2);  // 20% turbulence
        atm.enable_turbulence(true);

        const int N = 1000;
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        double sum_sq_x = 0.0, sum_sq_y = 0.0, sum_sq_z = 0.0;

        for (int i = 0; i < N; ++i) {
            Vec3 turb = atm.get_turbulent_wind();
            sum_x += turb.x;
            sum_y += turb.y;
            sum_z += turb.z;
            sum_sq_x += turb.x * turb.x;
            sum_sq_y += turb.y * turb.y;
            sum_sq_z += turb.z * turb.z;
        }

        double mean_x = sum_x / N;
        double mean_y = sum_y / N;
        double mean_z = sum_z / N;

        double std_x = std::sqrt((sum_sq_x - sum_x * sum_x / N) / N);
        double std_y = std::sqrt((sum_sq_y - sum_y * sum_y / N) / N);
        double std_z = std::sqrt((sum_sq_z - sum_z * sum_z / N) / N);

        std::cout << "Turbulence statistics from " << N << " samples:\n";
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "  Mean:   (" << mean_x << ", " << mean_y << ", " << mean_z << ")\n";
        std::cout << "  StdDev: (" << std_x << ", " << std_y << ", " << std_z << ")\n\n";

        std::cout << "Expected: Mean ≈ 0, StdDev ≈ " << (15.0 * 0.2 / std::sqrt(3.0)) << "\n";
        std::cout << "(Theoretical std dev for isotropic turbulence)\n";
    }

    // =========================================================================
    // Part 6: Wind Effect on Trajectory
    // =========================================================================
    {
        print_separator("PART 6: TRAJECTORY DEFLECTION");

        Atmosphere atm;

        // Clear day vs windy day comparison
        struct TrajectoryPoint {
            double x, y, z;
        };

        std::cout << "SIMPLIFIED TRAJECTORY COMPARISON\n\n";
        std::cout << "Launch: 45° elevation, 800 m/s initial velocity\n";
        std::cout << "Flight time: ~60 seconds\n\n";

        // No wind case (ideal)
        std::cout << "NO WIND:\n";
        std::cout << "  Impact point: (25000, 0, 0) m\n";
        std::cout << "  Lateral drift: 0 m\n\n";

        // With crosswind
        atm.set_wind(0.0, 10.0, 0.0);  // 10 m/s crosswind
        std::cout << "WITH 10 m/s CROSSWIND:\n";

        // Simplified estimate: drift = 0.5 * (v_wind / v_projectile) * g * t^2
        // More accurate would require full integration
        double flight_time = 60.0;
        double crosswind = 10.0;
        double avg_velocity = 400.0;  // Average over trajectory
        double drift_estimate = crosswind * flight_time * 0.7;  // Approximate factor

        std::cout << "  Impact point: (25000, " << std::fixed << std::setprecision(1)
                  << drift_estimate << ", 0) m\n";
        std::cout << "  Lateral drift: " << drift_estimate << " m\n\n";

        std::cout << "Note: Crosswind causes significant lateral deflection!\n";
        std::cout << "      Real artillery requires wind compensation.\n";
    }

    print_separator("DEMO COMPLETE");
    std::cout << "\nKey takeaways:\n";
    std::cout << "  1. Wind affects relative velocity, not ground speed\n";
    std::cout << "  2. Drag force depends on (v_projectile - v_wind)^2\n";
    std::cout << "  3. Headwinds increase drag, tailwinds decrease it\n";
    std::cout << "  4. Crosswinds cause lateral drift\n";
    std::cout << "  5. Turbulence adds stochastic variation to all wind components\n";
    std::cout << "  6. For long-range artillery, wind compensation is critical\n\n";

    return 0;
}
