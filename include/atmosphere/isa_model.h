#ifndef BALLISTX_ISA_MODEL_H
#define BALLISTX_ISA_MODEL_H

#include "utils/vec3.h"
#include <cmath>
#include <random>

namespace ballistx {

/**
 * @brief International Standard Atmosphere (ISA) model
 *
 * Calculates atmospheric properties (density, temperature, pressure)
 * based on altitude using the 1976 ISA model with standard atmospheric layers.
 *
 * **ISA Model Layers:**
 * - Troposphere (0-11 km): Temperature decreases linearly
 * - Tropopause (11 km): Constant temperature
 * - Stratosphere (11-20 km): Temperature increases slowly
 * - Stratosphere (20-32 km): Temperature constant
 *
 * **Physical Properties:**
 * - Sea level: 15°C (288.15 K), 101325 Pa, 1.225 kg/m³
 * - Tropopause: -56.5°C (216.65 K)
 * - Models realistic density variation for ballistic calculations
 *
 * **Applications:**
 * - Drag force calculation (ρ × v² × Cd × A)
 * - Speed of sound for Mach number (a = √(γ × R × T))
 * - Engine performance analysis
 * - Aerodynamic heating
 *
 * @example
 * @code
 * // Create atmosphere at sea level
 * Atmosphere sea_level(0.0);
 * std::cout << "Density: " << sea_level.get_density() << " kg/m³" << std::endl;
 * std::cout << "Speed of sound: " << sea_level.get_speed_of_sound() << " m/s" << std::endl;
 *
 * // Create atmosphere at 10 km altitude
 * Atmosphere high_altitude(10000.0);
 * double density_ratio = high_altitude.get_density() / sea_level.get_density();
 * std::cout << "Density ratio: " << density_ratio << std::endl;
 *
 * // Add wind
 * high_altitude.set_wind_velocity(Vec3(20.0, 0.0, 5.0));  // 20 m/s east, 5 m/s north
 *
 * // Calculate relative velocity for drag
 * Vec3 projectile_vel(300.0, 0.0, 0.0);
 * Vec3 relative_vel = high_altitude.get_relative_velocity(projectile_vel);
 * @endcode
 *
 * @see DragModel for drag force calculations
 */
class Atmosphere {
public:
    /**
     * @brief Sea level temperature
     *
     * Standard temperature at sea level: 15°C = 288.15 K
     */
    static constexpr double SEA_LEVEL_TEMP = 288.15;  ///< K (15°C)

    /**
     * @brief Sea level pressure
     *
     * Standard atmospheric pressure: 101325 Pa (1 atm)
     */
    static constexpr double SEA_LEVEL_PRESSURE = 101325.0;  ///< Pa

    /**
     * @brief Sea level air density
     *
     * Standard air density at sea level: 1.225 kg/m³
     */
    static constexpr double SEA_LEVEL_DENSITY = 1.225;  ///< kg/m³

    /**
     * @brief Temperature lapse rate in troposphere
     *
     * Temperature decrease per meter: 0.0065 K/m = 6.5 K/km
     */
    static constexpr double LAPSE_RATE_TROPOSPHERE = 0.0065;  ///< K/m

    /**
     * @brief Tropopause altitude
     *
     * Boundary between troposphere and stratosphere: 11 km
     */
    static constexpr double TROPOPAUSE_ALTITUDE = 11000.0;  ///< m

    /**
     * @brief Stratosphere temperature
     *
     * Constant temperature in lower stratosphere: -56.5°C = 216.65 K
     */
    static constexpr double STRATOSPHERE_TEMP = 216.65;  ///< K

    /**
     * @brief Specific gas constant for dry air
     *
     * R = 287.05 J/(kg·K)
     */
    static constexpr double GAS_CONSTANT = 287.05;  ///< J/(kg·K)

    /**
     * @brief Gravitational acceleration
     *
     * Standard gravity: 9.80665 m/s²
     */
    static constexpr double GRAVITY = 9.80665;  ///< m/s²

    /**
     * @brief Default constructor
     *
     * Creates atmosphere at sea level with standard conditions.
     */
    Atmosphere() = default;

    /**
     * @brief Create atmosphere at specific altitude
     *
     * Automatically calculates temperature, pressure, and density
     * for the given altitude using ISA model.
     *
     * @param altitude Altitude above sea level [m]
     *
     * @example
     * @code
     * Atmosphere at_10km(10000.0);  // 10 km altitude
     * @endcode
     */
    explicit Atmosphere(double altitude);

    /**
     * @brief Default destructor
     */
    ~Atmosphere() = default;

    /**
     * @brief Get temperature at current altitude
     *
     * @return Temperature [K]
     */
    double get_temperature() const { return temperature_; }

    /**
     * @brief Get pressure at current altitude
     *
     * @return Pressure [Pa]
     */
    double get_pressure() const { return pressure_; }

    /**
     * @brief Get air density at current altitude
     *
     * @return Density [kg/m³]
     */
    double get_density() const { return density_; }

    /**
     * @brief Get current altitude
     *
     * @return Altitude [m]
     */
    double get_altitude() const { return altitude_; }

    /**
     * @brief Get speed of sound at current altitude
     *
     * Calculates: a = √(γ × R × T)
     * where γ = 1.4 (ratio of specific heats for air)
     *
     * @return Speed of sound [m/s]
     *
     * @example
     * @code
     * Atmosphere sea_level(0.0);
     * double speed_of_sound = sea_level.get_speed_of_sound();  // ~340 m/s
     * @endcode
     */
    double get_speed_of_sound() const;

    // === WIND MODEL ===

    /**
     * @brief Get wind velocity vector
     *
     * @return Wind velocity [m/s]
     */
    Vec3 get_wind_velocity() const { return wind_velocity_; }

    /**
     * @brief Set wind velocity vector
     *
     * @param wind Wind velocity [m/s]
     *
     * @example
     * @code
     * Atmosphere atm(0.0);
     * atm.set_wind_velocity(Vec3(10.0, 0.0, 5.0));  // 10 m/s east, 5 m/s north
     * @endcode
     */
    void set_wind_velocity(const Vec3& wind) { wind_velocity_ = wind; }

    /**
     * @brief Set wind components individually
     *
     * @param speed_x X wind component [m/s]
     * @param speed_y Y wind component [m/s]
     * @param speed_z Z wind component [m/s]
     */
    void set_wind(double speed_x, double speed_y, double speed_z);

    // === TURBULENCE MODEL ===

    /**
     * @brief Get turbulent wind component
     *
     * Returns a random velocity perturbation based on Gaussian distribution.
     * Only active if turbulence is enabled.
     *
     * @return Turbulent wind velocity [m/s]
     */
    Vec3 get_turbulent_wind() const;

    /**
     * @brief Set turbulence intensity
     *
     * @param intensity Turbulence intensity (0.0 = none, 1.0 = 100%)
     *
     * @example
     * @code
     * Atmosphere atm(0.0);
     * atm.set_wind_velocity(Vec3(10.0, 0.0, 0.0));
     * atm.enable_turbulence(true);
     * atm.set_turbulence_intensity(0.2);  // 20% turbulence
     * @endcode
     */
    void set_turbulence_intensity(double intensity) { turbulence_intensity_ = intensity; }

    /**
     * @brief Get turbulence intensity
     *
     * @return Turbulence intensity (0.0-1.0)
     */
    double get_turbulence_intensity() const { return turbulence_intensity_; }

    /**
     * @brief Enable or disable turbulence
     *
     * @param enable true to enable, false to disable
     */
    void enable_turbulence(bool enable) { turbulence_enabled_ = enable; }

    /**
     * @brief Check if turbulence is enabled
     *
     * @return true if enabled, false otherwise
     */
    bool is_turbulence_enabled() const { return turbulence_enabled_; }

    // === RELATIVE VELOCITY ===

    /**
     * @brief Calculate relative velocity including wind and turbulence
     *
     * Returns: v_projectile - v_wind - v_turbulence
     * Used for drag force calculations.
     *
     * @param projectile_velocity Projectile velocity [m/s]
     * @return Relative velocity [m/s]
     *
     * @example
     * @code
     * Atmosphere atm(5000.0);
     * atm.set_wind_velocity(Vec3(15.0, 0.0, 0.0));
     * Vec3 v_rel = atm.get_relative_velocity(Vec3(300.0, 0.0, 0.0));
     * // v_rel ≈ (285, 0, 0) m/s
     * @endcode
     */
    Vec3 get_relative_velocity(const Vec3& projectile_velocity) const;

    /**
     * @brief Get airspeed (magnitude of relative velocity)
     *
     * @param ground_speed Velocity relative to ground [m/s]
     * @return Airspeed [m/s]
     */
    Vec3 get_airspeed(const Vec3& ground_speed) const;

    /**
     * @brief Set altitude and recalculate atmospheric properties
     *
     * Updates temperature, pressure, and density for new altitude.
     *
     * @param altitude New altitude [m]
     */
    void set_altitude(double altitude);

    // === STATIC CALCULATION METHODS ===

    /**
     * @brief Calculate temperature at altitude
     *
     * Static method for temperature calculation without creating object.
     *
     * @param altitude Altitude [m]
     * @return Temperature [K]
     *
     * @example
     * @code
     * double temp = Atmosphere::calculate_temperature(10000.0);  // ~223 K
     * @endcode
     */
    static double calculate_temperature(double altitude);

    /**
     * @brief Calculate pressure at altitude
     *
     * Static method for pressure calculation.
     *
     * @param altitude Altitude [m]
     * @return Pressure [Pa]
     */
    static double calculate_pressure(double altitude);

    /**
     * @brief Calculate air density at altitude
     *
     * Static method for density calculation.
     *
     * @param altitude Altitude [m]
     * @return Density [kg/m³]
     */
    static double calculate_density(double altitude);

    /**
     * @brief Calculate speed of sound for given temperature
     *
     * @param temperature Temperature [K]
     * @return Speed of sound [m/s]
     */
    static double calculate_speed_of_sound(double temperature);

private:
    double altitude_ = 0.0;                    ///< Altitude [m]
    double temperature_ = SEA_LEVEL_TEMP;      ///< Temperature [K]
    double pressure_ = SEA_LEVEL_PRESSURE;     ///< Pressure [Pa]
    double density_ = SEA_LEVEL_DENSITY;       ///< Density [kg/m³]
    Vec3 wind_velocity_ = Vec3::zero();        ///< Wind velocity [m/s]

    // Turbulence parameters
    double turbulence_intensity_ = 0.1;        ///< Turbulence intensity (0-1)
    bool turbulence_enabled_ = false;           ///< Turbulence enabled flag
    mutable std::mt19937 rng_{std::random_device{}()};  ///< Random generator
    mutable std::normal_distribution<double> dist_{0.0, 1.0};  ///< Normal dist

    /**
     * @brief Update atmospheric properties for current altitude
     */
    void update_atmosphere();
};

} // namespace ballistx

#endif // BALLISTX_ISA_MODEL_H
