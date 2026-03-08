/**
 * @file projectile_types_demo.cpp
 * @brief Demonstration of projectile type system
 *
 * Shows different ammunition types, their physical properties,
 * and comparative ballistic characteristics.
 */

#include "ballistics/projectile.h"
#include "atmosphere/isa_model.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ballistx;

void print_separator(const std::string& title) {
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(70, '=') << "\n";
}

void print_header(const std::string& title) {
    std::cout << "\n" << std::string(70, '-') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(70, '-') << "\n\n";
}

// Calculate ballistic coefficient: BC = m / (Cd * A)
double calculate_ballistic_coefficient(const Projectile& proj) {
    double area = proj.get_area();
    double cd = proj.get_drag_coefficient();
    double mass = proj.get_mass();
    return mass / (cd * area);
}

// Calculate sectional density: SD = m / A^1.5 (for caliber-normalized comparison)
double calculate_sectional_density(const Projectile& proj) {
    double diameter = proj.get_diameter();
    double area = proj.get_area();
    double mass = proj.get_mass();
    return mass / std::pow(area, 1.5);
}

int main() {
    print_separator("PROJECTILE TYPES DATABASE DEMO");

    // ========================================================================
    // Part 1: Artillery Shells
    // ========================================================================
    {
        print_header("PART 1: ARTILLERY SHELLS");

        std::vector<ProjectileType> types = {
            ProjectileType::ARTILLERY_155MM,
            ProjectileType::ARTILLERY_105MM,
            ProjectileType::ARTILLERY_120MM_MORTAR,
            ProjectileType::ARTILLERY_81MM_MORTAR,
            ProjectileType::ARTILLERY_60MM_MORTAR
        };

        std::cout << std::left << std::setw(25) << "Type"
                  << std::setw(10) << "Mass(kg)"
                  << std::setw(12) << "Diam(mm)"
                  << std::setw(10) << "Cd"
                  << std::setw(12) << "Area(cm²)"
                  << "\n";
        std::cout << std::string(70, '-') << "\n";

        for (auto type : types) {
            Projectile proj = Projectile::create(type);
            auto props = Projectile::get_properties(type);

            std::cout << std::left << std::setw(25) << props.name
                      << std::fixed << std::setprecision(1)
                      << std::setw(10) << props.mass
                      << std::setw(12) << (props.diameter * 1000.0)
                      << std::setprecision(2)
                      << std::setw(10) << props.drag_coefficient
                      << std::setprecision(1)
                      << std::setw(12) << (proj.get_area() * 10000.0)
                      << "\n";
        }

        std::cout << "\nComparison:\n";
        std::cout << "  155mm has 2.8x the mass of 105mm but only 2.2x the area\n";
        std::cout << "  Mortars have higher drag (less streamlined)\n";
    }

    // ========================================================================
    // Part 2: Rockets
    // ========================================================================
    {
        print_header("PART 2: UNGUIDED ROCKETS");

        std::vector<ProjectileType> types = {
            ProjectileType::ROCKET_70MM,
            ProjectileType::ROCKET_122MM,
            ProjectileType::ROCKET_107MM,
            ProjectileType::ROCKET_57MM
        };

        std::cout << std::left << std::setw(25) << "Rocket Type"
                  << std::setw(10) << "Mass(kg)"
                  << std::setw(12) << "Diam(mm)"
                  << std::setw(10) << "Cd"
                  << std::setw(15) << "BC"
                  << "\n";
        std::cout << std::string(70, '-') << "\n";

        for (auto type : types) {
            Projectile proj = Projectile::create(type);
            auto props = Projectile::get_properties(type);
            double bc = calculate_ballistic_coefficient(proj);

            std::cout << std::left << std::setw(25) << props.name
                      << std::fixed << std::setprecision(1)
                      << std::setw(10) << props.mass
                      << std::setw(12) << (props.diameter * 1000.0)
                      << std::setprecision(2)
                      << std::setw(10) << props.drag_coefficient
                      << std::setprecision(2)
                      << std::setw(15) << bc
                      << "\n";
        }

        std::cout << "\nNote: BC = Ballistic Coefficient (higher = better aerodynamics)\n";
        std::cout << "      122mm Grad has highest BC due to favorable mass/area ratio\n";
    }

    // ========================================================================
    // Part 3: Missiles
    // ========================================================================
    {
        print_header("PART 3: GUIDED MISSILES");

        std::vector<ProjectileType> types = {
            ProjectileType::MISSILE_AIR_TO_AIR,
            ProjectileType::MISSILE_AIR_TO_GROUND,
            ProjectileType::MISSILE_SURFACE_TO_AIR,
            ProjectileType::MISSILE_ANTI_TANK
        };

        std::cout << std::left << std::setw(25) << "Missile Type"
                  << std::setw(10) << "Mass(kg)"
                  << std::setw(12) << "Diam(mm)"
                  << std::setw(10) << "Cd"
                  << "\n";
        std::cout << std::string(70, '-') << "\n";

        for (auto type : types) {
            Projectile proj = Projectile::create(type);
            auto props = Projectile::get_properties(type);

            std::cout << std::left << std::setw(25) << props.name
                      << std::fixed << std::setprecision(1)
                      << std::setw(10) << props.mass
                      << std::setw(12) << (props.diameter * 1000.0)
                      << std::setprecision(2)
                      << std::setw(10) << props.drag_coefficient
                      << "\n";
        }

        std::cout << "\nObservations:\n";
        std::cout << "  AAM (air-to-air) has lowest Cd - most streamlined\n";
        std::cout << "  SAM (surface-to-air) is heaviest at 900kg\n";
        std::cout << "  ATGM (anti-tank) is lightest at 11.8kg\n";
    }

    // ========================================================================
    // Part 4: Small Arms Comparison
    // ========================================================================
    {
        print_header("PART 4: SMALL ARMS AMMUNITION");

        std::vector<ProjectileType> types = {
            ProjectileType::BULLET_5_56MM,
            ProjectileType::BULLET_7_62MM,
            ProjectileType::BULLET_12_7MM
        };

        std::cout << std::left << std::setw(20) << "Cartridge"
                  << std::setw(12) << "Mass(g)"
                  << std::setw(12) << "Diam(mm)"
                  << std::setw(12) << "Area(mm²)"
                  << std::setw(12) << "SD"
                  << "\n";
        std::cout << std::string(70, '-') << "\n";

        for (auto type : types) {
            Projectile proj = Projectile::create(type);
            auto props = Projectile::get_properties(type);
            double sd = calculate_sectional_density(proj);

            std::cout << std::left << std::setw(20) << props.name
                      << std::fixed << std::setprecision(1)
                      << std::setw(12) << (props.mass * 1000.0)
                      << std::setw(12) << (props.diameter * 1000.0)
                      << std::setprecision(2)
                      << std::setw(12) << (proj.get_area() * 1e6)
                      << std::setprecision(3)
                      << std::setw(12) << sd
                      << "\n";
        }

        std::cout << "\nNote: SD = Sectional Density (penetration potential)\n";
        std::cout << "      .50 BMG has 11.5x the mass of 5.56mm with only 5x the diameter\n";
    }

    // ========================================================================
    // Part 5: Tank Rounds
    // ========================================================================
    {
        print_header("PART 5: TANK AMMUNITION (120mm)");

        std::vector<ProjectileType> types = {
            ProjectileType::APFSDS_120MM,
            ProjectileType::HEAT_120MM
        };

        for (auto type : types) {
            Projectile proj = Projectile::create(type);
            auto props = Projectile::get_properties(type);

            std::cout << props.name << ":\n";
            std::cout << "  Mass: " << std::fixed << std::setprecision(1)
                      << props.mass << " kg\n";
            std::cout << "  Diameter: " << (props.diameter * 1000.0) << " mm\n";
            std::cout << "  Drag Coefficient: " << std::setprecision(2)
                      << props.drag_coefficient << "\n";
            std::cout << "  " << props.description << "\n\n";
        }

        std::cout << "Key difference:\n";
        std::cout << "  APFSDS: 25mm penetrator (discarding sabot) - very low drag\n";
        std::cout << "  HEAT: 120mm full caliber - standard shell drag\n";
    }

    // ========================================================================
    // Part 6: Drag Force Comparison
    // ========================================================================
    {
        print_header("PART 6: DRAG FORCE COMPARISON @ Mach 2");

        Atmosphere atm(0.0);
        double density = atm.get_density();
        double mach = 2.0;
        double speed_of_sound = atm.get_speed_of_sound();
        double velocity = mach * speed_of_sound;

        std::cout << "Conditions: Sea level, Mach " << mach
                  << " (" << std::fixed << std::setprecision(0)
                  << velocity << " m/s)\n";
        std::cout << "Air density: " << std::setprecision(3)
                  << density << " kg/m³\n\n";

        std::cout << std::left << std::setw(25) << "Projectile"
                  << std::setw(15) << "Area(m²)"
                  << std::setw(15) << "Drag(N)"
                  << std::setw(15) << "Decel(m/s²)"
                  << "\n";
        std::cout << std::string(70, '-') << "\n";

        std::vector<ProjectileType> types = {
            ProjectileType::ARTILLERY_155MM,
            ProjectileType::ROCKET_70MM,
            ProjectileType::MISSILE_AIR_TO_AIR,
            ProjectileType::BULLET_12_7MM
        };

        for (auto type : types) {
            Projectile proj = Projectile::create(type);
            auto props = Projectile::get_properties(type);

            double area = proj.get_area();
            double cd = props.drag_coefficient;
            double drag = 0.5 * density * velocity * velocity * cd * area;
            double deceleration = drag / props.mass;

            std::cout << std::left << std::setw(25) << props.name
                      << std::scientific
                      << std::setw(15) << area
                      << std::setw(15) << drag
                      << std::fixed << std::setprecision(1)
                      << std::setw(15) << deceleration
                      << "\n";
        }

        std::cout << "\nNote: Deceleration = Drag / Mass\n";
        std::cout << "      Lower deceleration = better velocity retention\n";
    }

    // ========================================================================
    // Part 7: Ballistic Coefficient Ranking
    // ========================================================================
    {
        print_header("PART 7: BALLISTIC COEFFICIENT RANKING");

        std::vector<ProjectileType> all_types = {
            ProjectileType::ARTILLERY_155MM,
            ProjectileType::ARTILLERY_105MM,
            ProjectileType::ROCKET_122MM,
            ProjectileType::MISSILE_AIR_TO_AIR,
            ProjectileType::APFSDS_120MM,
            ProjectileType::BULLET_12_7MM,
            ProjectileType::BULLET_5_56MM
        };

        std::vector<std::pair<std::string, double>> bc_list;

        for (auto type : all_types) {
            Projectile proj = Projectile::create(type);
            auto props = Projectile::get_properties(type);
            double bc = calculate_ballistic_coefficient(proj);
            bc_list.push_back({props.name, bc});
        }

        // Sort by BC (descending)
        std::sort(bc_list.begin(), bc_list.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });

        std::cout << "Ranking (higher BC = better aerodynamics):\n\n";
        std::cout << std::left << std::setw(25) << "Projectile"
                  << std::setw(20) << "Ballistic Coeff"
                  << "Relative to .50 BMG\n";
        std::cout << std::string(70, '-') << "\n";

        double b50_caliber = 0.0;
        for (const auto& [name, bc] : bc_list) {
            if (name.find("12.7x99mm") != std::string::npos) {
                b50_caliber = bc;
                break;
            }
        }

        for (const auto& [name, bc] : bc_list) {
            double relative = (b50_caliber > 0) ? (bc / b50_caliber) : 1.0;

            std::cout << std::left << std::setw(25) << name
                      << std::fixed << std::setprecision(2)
                      << std::setw(20) << bc
                      << std::setprecision(1)
                      << relative << "x\n";
        }
    }

    // ========================================================================
    // Part 8: Custom Projectile Creation
    // ========================================================================
    {
        print_header("PART 8: CUSTOM PROJECTILE CREATION");

        // Method 1: Using ProjectileProperties
        ProjectileProperties custom_props;
        custom_props.mass = 25.0;              // kg
        custom_props.diameter = 0.130;         // m (130mm)
        custom_props.drag_coefficient = 0.27;
        custom_props.name = "130mm SPH-1";
        custom_props.description = "Custom self-propelled howitzer round";

        Projectile custom1 = Projectile::create(custom_props);
        std::cout << "Custom projectile created:\n";
        std::cout << "  Name: " << custom1.get_name() << "\n";
        std::cout << "  Mass: " << custom1.get_mass() << " kg\n";
        std::cout << "  Diameter: " << (custom1.get_diameter() * 1000.0) << " mm\n";
        std::cout << "  Is custom: " << (custom1.is_custom() ? "Yes" : "No") << "\n\n";

        // Method 2: Direct constructor
        Projectile custom2(50.0, 0.155, 0.23);
        custom2.set_initial_velocity(850.0);
        std::cout << "Direct constructor projectile:\n";
        std::cout << "  Mass: " << custom2.get_mass() << " kg\n";
        std::cout << "  Initial velocity: " << custom2.get_initial_velocity() << " m/s\n";
    }

    // ========================================================================
    // Part 9: Type Information
    // ========================================================================
    {
        print_header("PART 9: TYPE INFORMATION SYSTEM");

        std::vector<ProjectileType> types = {
            ProjectileType::ARTILLERY_155MM,
            ProjectileType::ROCKET_70MM,
            ProjectileType::MISSILE_AIR_TO_AIR
        };

        for (auto type : types) {
            std::cout << "Enum name: " << Projectile::get_type_name(type) << "\n";

            auto props = Projectile::get_properties(type);
            std::cout << "Display name: " << props.name << "\n";
            std::cout << "Description: " << props.description << "\n";
            std::cout << "\n";
        }
    }

    // ========================================================================
    // Part 10: Practical Application - Range Estimation
    // ========================================================================
    {
        print_header("PART 10: SIMPLIFIED RANGE COMPARISON");

        std::cout << "Launch angle: 45°, Initial velocity: 800 m/s\n\n";

        std::vector<ProjectileType> types = {
            ProjectileType::ARTILLERY_155MM,
            ProjectileType::ARTILLERY_105MM,
            ProjectileType::ROCKET_122MM
        };

        std::cout << std::left << std::setw(25) << "Projectile"
                  << std::setw(15) << "Mass(kg)"
                  << std::setw(15) << "Est.Range(km)"
                  << "\n";
        std::cout << std::string(70, '-') << "\n";

        for (auto type : types) {
            Projectile proj = Projectile::create(type);
            auto props = Projectile::get_properties(type);

            // Very rough vacuum range estimate (no drag)
            // R = v²/g * sin(2θ)
            double v = 800.0;
            double theta = 45.0 * M_PI / 180.0;
            double vacuum_range = (v * v / 9.80665) * std::sin(2.0 * theta);

            // Rough drag correction factor based on BC
            double bc = calculate_ballistic_coefficient(proj);
            double drag_factor = std::exp(-vacuum_range / (bc * 10000.0));
            double estimated_range = vacuum_range * drag_factor;

            std::cout << std::left << std::setw(25) << props.name
                      << std::fixed << std::setprecision(1)
                      << std::setw(15) << props.mass
                      << std::setprecision(1)
                      << std::setw(15) << (estimated_range / 1000.0)
                      << "\n";
        }

        std::cout << "\nNote: These are rough estimates.\n";
        std::cout << "      Actual range depends on drag curve, atmosphere, etc.\n";
    }

    print_separator("DEMO COMPLETE");

    std::cout << "\nKey features demonstrated:\n";
    std::cout << "  1. 20+ predefined projectile types with real-world data\n";
    std::cout << "  2. Factory methods: Projectile::create(type)\n";
    std::cout << "  3. Custom projectile creation via ProjectileProperties\n";
    std::cout << "  4. Ballistic coefficient and sectional density calculations\n";
    std::cout << "  5. Type information and naming system\n";
    std::cout << "  6. Comparative analysis across ammunition categories\n\n";

    return 0;
}
