#ifndef BALLISTX_PROJECTILE_H
#define BALLISTX_PROJECTILE_H

namespace ballistx {

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

    // Getters
    double get_mass() const { return mass_; }
    double get_diameter() const { return diameter_; }
    double get_drag_coefficient() const { return drag_coefficient_; }
    double get_area() const { return area_; }
    double get_initial_velocity() const { return initial_velocity_; }
    double get_launch_angle() const { return launch_angle_; }

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

private:
    double mass_ = 0.0;           // kg
    double diameter_ = 0.0;       // m
    double drag_coefficient_ = 0.0; // dimensionless
    double area_ = 0.0;           // m^2 (cross-sectional area)
    double initial_velocity_ = 0.0;  // m/s
    double launch_angle_ = 0.0;   // radians
};

} // namespace ballistx

#endif // BALLISTX_PROJECTILE_H
