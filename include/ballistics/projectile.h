#ifndef BALLISTX_PROJECTILE_H
#define BALLISTX_PROJECTILE_H

#include <string>

namespace ballistx {

/**
 * @brief Enumeration of standard projectile types
 *
 * Predefined projectile types with real-world physical properties.
 * Each type has specific mass, diameter, and drag characteristics
 * based on actual ammunition data.
 *
 * **Categories:**
 * - Artillery: Large caliber indirect fire weapons
 * - Rockets: Unguided rocket projectiles
 * - Missiles: Guided munitions
 * - Small arms: Hand-held weapon ammunition
 * - Tank rounds: Anti-armor projectiles
 *
 * @example
 * @code
 * Projectile p = Projectile::create(ProjectileType::ARTILLERY_155MM);
 * std::cout << "Mass: " << p.get_mass() << " kg" << std::endl;
 * @endcode
 */
enum class ProjectileType {
    // Artillery shells
    ARTILLERY_155MM,          ///< Standard 155mm howitzer shell (NATO)
    ARTILLERY_105MM,          ///< 105mm light artillery
    ARTILLERY_120MM_MORTAR,   ///< 120mm mortar shell
    ARTILLERY_81MM_MORTAR,    ///< 81mm mortar shell
    ARTILLERY_60MM_MORTAR,    ///< 60mm mortar shell

    // Rockets
    ROCKET_70MM,              ///< 70mm Hydra rocket (unguided)
    ROCKET_122MM,             ///< 122mm Grad rocket
    ROCKET_107MM,             ///< 107mm rocket
    ROCKET_57MM,              ///< 57mm rocket

    // Missiles
    MISSILE_AIR_TO_AIR,       ///< Medium-range AAM (e.g., AMRAAM)
    MISSILE_AIR_TO_GROUND,    ///< AGM (e.g., Maverick)
    MISSILE_SURFACE_TO_AIR,   ///< SAM (e.g., Patriot)
    MISSILE_ANTI_TANK,        ///< ATGM (e.g., Javelin, TOW)

    // Small arms
    BULLET_5_56MM,            ///< 5.56x45mm NATO
    BULLET_7_62MM,            ///< 7.62x51mm NATO
    BULLET_12_7MM,            ///< 12.7x99mm (.50 BMG)

    // Tank rounds
    APFSDS_120MM,             ///< Armor-piercing fin-stabilized discarding sabot
    HEAT_120MM,               ///< High-explosive anti-tank

    // Custom
    CUSTOM                    ///< User-defined properties
};

/**
 * @brief Physical properties for a projectile type
 *
 * Contains all physical characteristics needed for ballistic simulation.
 *
 * @example
 * @code
 * ProjectileProperties props;
 * props.mass = 50.0;
 * props.diameter = 0.155;
 * props.drag_coefficient = 0.3;
 * props.name = "Custom Shell";
 * props.description = "155mm HE projectile";
 *
 * Projectile p = Projectile::create(props);
 * @endcode
 */
struct ProjectileProperties {
    double mass;              ///< Mass [kg]
    double diameter;          ///< Diameter [m]
    double drag_coefficient;  ///< Drag coefficient (subsonic reference)
    std::string name;         ///< Human-readable name
    std::string description;  ///< Additional info
};

/**
 * @brief Projectile class for ballistic simulation
 *
 * Represents a projectile with physical properties for trajectory calculation.
 * All units are in SI (kg, meters, seconds, radians).
 *
 * **Physical Properties:**
 * - Mass: Affects inertia and momentum
 * - Diameter: Determines cross-sectional area for drag
 * - Drag coefficient: Aerodynamic resistance (Mach-dependent)
 *
 * **Calculated Properties:**
 * - Cross-sectional area: A = π × (d/2)²
 * - Mach number: M = v / c (where c is speed of sound)
 *
 * @example
 * @code
 * // Create standard 155mm artillery shell
 * Projectile shell = Projectile::create(ProjectileType::ARTILLERY_155MM);
 *
 * // Create custom projectile
 * Projectile custom;
 * custom.set_mass(45.0);              // 45 kg
 * custom.set_diameter(0.155);          // 155 mm
 * custom.set_drag_coefficient(0.3);    // Cd = 0.3
 *
 * // Calculate properties
 * double area = shell.get_area();                  // Cross-sectional area
 * double mach = shell.get_mach_number(500.0);      // At 500 m/s
 *
 * // Check validity
 * if (shell.is_valid()) {
 *     std::cout << "Valid projectile" << std::endl;
 * }
 * @endcode
 *
 * @see DragModel for drag force calculations
 * @see Atmosphere for air density and speed of sound
 */
class Projectile {
public:
    /**
     * @brief Default constructor
     *
     * Creates an invalid projectile with zero properties.
     * Use set_*() methods or create() factory to initialize.
     */
    Projectile() = default;

    /**
     * @brief Parameterized constructor
     *
     * Creates a projectile with specified physical properties.
     *
     * @param mass Mass [kg]
     * @param diameter Diameter [m]
     * @param drag_coefficient Drag coefficient (dimensionless)
     *
     * @example
     * @code
     * Projectile p(50.0, 0.155, 0.3);  // 155mm shell
     * @endcode
     */
    Projectile(double mass, double diameter, double drag_coefficient);

    /**
     * @brief Create projectile from standard type
     *
     * Factory method that creates a projectile with predefined
     * physical properties based on real-world ammunition.
     *
     * @param type Projectile type from ProjectileType enum
     * @return Projectile with initialized properties
     *
     * @example
     * @code
     * Projectile shell = Projectile::create(ProjectileType::ARTILLERY_155MM);
     * Projectile rocket = Projectile::create(ProjectileType::ROCKET_70MM);
     * Projectile bullet = Projectile::create(ProjectileType::BULLET_7_62MM);
     * @endcode
     */
    static Projectile create(ProjectileType type);

    /**
     * @brief Create projectile from custom properties
     *
     * Factory method for creating a projectile with user-defined properties.
     *
     * @param props ProjectileProperties structure
     * @return Projectile with specified properties
     *
     * @example
     * @code
     * ProjectileProperties props;
     * props.mass = 100.0;
     * props.diameter = 0.120;
     * props.drag_coefficient = 0.25;
     * props.name = "Custom Rocket";
     *
     * Projectile p = Projectile::create(props);
     * @endcode
     */
    static Projectile create(const ProjectileProperties& props);

    /**
     * @brief Get properties for projectile type
     *
     * Returns the physical properties for a standard projectile type.
     *
     * @param type Projectile type
     * @return ProjectileProperties structure
     */
    static ProjectileProperties get_properties(ProjectileType type);

    /**
     * @brief Get human-readable type name
     *
     * Returns the name of a projectile type as a string.
     *
     * @param type Projectile type
     * @return Type name string
     */
    static std::string get_type_name(ProjectileType type);

    /**
     * @brief Get projectile mass
     *
     * @return Mass [kg]
     */
    double get_mass() const { return mass_; }

    /**
     * @brief Get projectile diameter
     *
     * @return Diameter [m]
     */
    double get_diameter() const { return diameter_; }

    /**
     * @brief Get drag coefficient
     *
     * Returns the subsonic reference drag coefficient.
     * Note: Actual drag varies with Mach number.
     *
     * @return Drag coefficient (dimensionless)
     */
    double get_drag_coefficient() const { return drag_coefficient_; }

    /**
     * @brief Get cross-sectional area
     *
     * Returns the frontal area: A = π × (d/2)²
     *
     * @return Cross-sectional area [m²]
     */
    double get_area() const { return area_; }

    /**
     * @brief Get initial velocity
     *
     * @return Initial velocity [m/s]
     */
    double get_initial_velocity() const { return initial_velocity_; }

    /**
     * @brief Get launch angle
     *
     * @return Launch angle [radians]
     */
    double get_launch_angle() const { return launch_angle_; }

    /**
     * @brief Get projectile type
     *
     * @return ProjectileType enum value
     */
    ProjectileType get_type() const { return type_; }

    /**
     * @brief Set projectile mass
     *
     * @param mass Mass [kg]
     */
    void set_mass(double mass) { mass_ = mass; }

    /**
     * @brief Set projectile diameter
     *
     * Automatically recalculates cross-sectional area.
     *
     * @param diameter Diameter [m]
     */
    void set_diameter(double diameter);

    /**
     * @brief Set drag coefficient
     *
     * @param cd Drag coefficient (dimensionless)
     */
    void set_drag_coefficient(double cd) { drag_coefficient_ = cd; }

    /**
     * @brief Set initial velocity
     *
     * @param v Velocity [m/s]
     */
    void set_initial_velocity(double v) { initial_velocity_ = v; }

    /**
     * @brief Set launch angle
     *
     * @param angle Angle [radians]
     */
    void set_launch_angle(double angle) { launch_angle_ = angle; }

    /**
     * @brief Get cross-sectional area
     *
     * Alias for get_area(). Returns the frontal area of the projectile.
     *
     * @return Cross-sectional area [m²]
     */
    double get_cross_sectional_area() const { return area_; }

    /**
     * @brief Calculate Mach number for given velocity
     *
     * Returns the ratio of velocity to speed of sound.
     * M = v / c where c is the speed of sound (default 343 m/s).
     *
     * @param velocity Velocity [m/s]
     * @param speed_of_sound Speed of sound [m/s] (default: 343 m/s at sea level)
     * @return Mach number (dimensionless)
     *
     * @example
     * @code
     * Projectile p = Projectile::create(ProjectileType::ARTILLERY_155MM);
     * double mach = p.get_mach_number(500.0);  // M ≈ 1.46 (supersonic)
     * @endcode
     */
    double get_mach_number(double velocity, double speed_of_sound = 343.0) const;

    /**
     * @brief Check if projectile has valid properties
     *
     * Returns true if mass, diameter, and drag coefficient are positive.
     *
     * @return true if valid, false otherwise
     *
     * @example
     * @code
     * Projectile p(50.0, 0.155, 0.3);
     * if (p.is_valid()) {
     *     std::cout << "Valid projectile" << std::endl;
     * }
     * @endcode
     */
    bool is_valid() const;

    /**
     * @brief Get projectile name
     *
     * Returns the human-readable name of the projectile.
     *
     * @return Name string
     */
    std::string get_name() const;

    /**
     * @brief Check if this is a custom projectile
     *
     * Returns true if the projectile was created with custom properties
     * rather than from a standard type.
     *
     * @return true if custom, false if standard type
     */
    bool is_custom() const { return type_ == ProjectileType::CUSTOM; }

private:
    double mass_ = 0.0;              ///< Mass [kg]
    double diameter_ = 0.0;          ///< Diameter [m]
    double drag_coefficient_ = 0.0;  ///< Drag coefficient
    double area_ = 0.0;              ///< Cross-sectional area [m²]
    double initial_velocity_ = 0.0;  ///< Initial velocity [m/s]
    double launch_angle_ = 0.0;      ///< Launch angle [rad]
    ProjectileType type_ = ProjectileType::CUSTOM;  ///< Projectile type
    std::string custom_name_;        ///< Name for custom projectiles
};

} // namespace ballistx

#endif // BALLISTX_PROJECTILE_H
