#ifndef BALLISTX_PROJECTILE_H
#define BALLISTX_PROJECTILE_H

#include <string>

namespace ballistx {

/**
 * @brief Enumeration of standard projectile types
 *
 * Each type has predefined physical properties based on real-world ammunition.
 */
enum class ProjectileType {
    // Artillery shells
    ARTILLERY_155MM,          // Standard 155mm howitzer shell (NATO)
    ARTILLERY_105MM,          // 105mm light artillery
    ARTILLERY_120MM_MORTAR,   // 120mm mortar shell
    ARTILLERY_81MM_MORTAR,    // 81mm mortar shell
    ARTILLERY_60MM_MORTAR,    // 60mm mortar shell

    // Rockets
    ROCKET_70MM,              // 70mm Hydra rocket (unguided)
    ROCKET_122MM,             // 122mm Grad rocket
    ROCKET_107MM,             // 107mm rocket
    ROCKET_57MM,              // 57mm rocket

    // Missiles
    MISSILE_AIR_TO_AIR,       // Medium-range AAM (e.g., AMRAAM)
    MISSILE_AIR_TO_GROUND,    // AGM (e.g., Maverick)
    MISSILE_SURFACE_TO_AIR,   // SAM (e.g., Patriot)
    MISSILE_ANTI_TANK,        // ATGM (e.g., Javelin, TOW)

    // Small arms
    BULLET_5_56MM,            // 5.56x45mm NATO
    BULLET_7_62MM,            // 7.62x51mm NATO
    BULLET_12_7MM,            // 12.7x99mm (.50 BMG)

    // Tank rounds
    APFSDS_120MM,             // Armor-piercing fin-stabilized discarding sabot
    HEAT_120MM,               // High-explosive anti-tank

    // Custom
    CUSTOM                    // User-defined properties
};

/**
 * @brief Physical properties for a projectile type
 */
struct ProjectileProperties {
    double mass;              // kg
    double diameter;          // m
    double drag_coefficient;  // dimensionless (subsonic reference)
    std::string name;         // Human-readable name
    std::string description;  // Additional info
};

/**
 * @brief Projectile class for ballistic simulation
 *
 * Represents a projectile with physical properties for trajectory calculation.
 * Units: SI (kg, meters, seconds, radians)
 */
class Projectile {
public:
    // Constructors
    Projectile() = default;
    Projectile(double mass, double diameter, double drag_coefficient);

    // Factory methods for standard projectile types
    static Projectile create(ProjectileType type);
    static Projectile create(const ProjectileProperties& props);

    // Get properties for projectile type
    static ProjectileProperties get_properties(ProjectileType type);
    static std::string get_type_name(ProjectileType type);

    // Getters
    double get_mass() const { return mass_; }
    double get_diameter() const { return diameter_; }
    double get_drag_coefficient() const { return drag_coefficient_; }
    double get_area() const { return area_; }
    double get_initial_velocity() const { return initial_velocity_; }
    double get_launch_angle() const { return launch_angle_; }
    ProjectileType get_type() const { return type_; }

    // Setters
    void set_mass(double mass) { mass_ = mass; }
    void set_diameter(double diameter);
    void set_drag_coefficient(double cd) { drag_coefficient_ = cd; }
    void set_initial_velocity(double v) { initial_velocity_ = v; }
    void set_launch_angle(double angle) { launch_angle_ = angle; }

    // Calculated properties
    double get_cross_sectional_area() const { return area_; }
    double get_mach_number(double velocity, double speed_of_sound = 343.0) const;

    // Validation
    bool is_valid() const;

    // Type info
    std::string get_name() const;
    bool is_custom() const { return type_ == ProjectileType::CUSTOM; }

private:
    double mass_ = 0.0;           // kg
    double diameter_ = 0.0;       // m
    double drag_coefficient_ = 0.0; // dimensionless
    double area_ = 0.0;           // m^2 (cross-sectional area)
    double initial_velocity_ = 0.0;  // m/s
    double launch_angle_ = 0.0;   // radians
    ProjectileType type_ = ProjectileType::CUSTOM;
    std::string custom_name_;     // For custom projectiles
};

} // namespace ballistx

#endif // BALLISTX_PROJECTILE_H
