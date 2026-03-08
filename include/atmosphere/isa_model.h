#ifndef BALLISTX_ISA_MODEL_H
#define BALLISTX_ISA_MODEL_H

#include "utils/vec3.h"
#include <cmath>
#include <random>

namespace ballistx {

/**
 * @brief International Standard Atmosphere (ISA) model
 *
 * Calculates air density, temperature, pressure based on altitude.
 * Uses the 1976 ISA model with standard atmospheric layers.
 *
 * Units: SI (Kelvin, Pa, kg/m^3, m/s, meters)
 */
class Atmosphere {
public:
    // Constants
    static constexpr double SEA_LEVEL_TEMP = 288.15;        // K (15°C)
    static constexpr double SEA_LEVEL_PRESSURE = 101325.0;  // Pa
    static constexpr double SEA_LEVEL_DENSITY = 1.225;      // kg/m^3
    static constexpr double LAPSE_RATE_TROPOSPHERE = 0.0065; // K/m
    static constexpr double TROPOPAUSE_ALTITUDE = 11000.0;  // m
    static constexpr double STRATOSPHERE_TEMP = 216.65;     // K
    static constexpr double GAS_CONSTANT = 287.05;          // J/(kg·K) for dry air
    static constexpr double GRAVITY = 9.80665;              // m/s^2

    // Constructors
    Atmosphere() = default;
    explicit Atmosphere(double altitude);
    ~Atmosphere() = default;

    // Get atmospheric properties at current altitude
    double get_temperature() const { return temperature_; }
    double get_pressure() const { return pressure_; }
    double get_density() const { return density_; }
    double get_altitude() const { return altitude_; }
    double get_speed_of_sound() const;

    // Wind model
    Vec3 get_wind_velocity() const { return wind_velocity_; }
    void set_wind_velocity(const Vec3& wind) { wind_velocity_ = wind; }
    void set_wind(double speed_x, double speed_y, double speed_z);

    // Turbulence model (Gaussian noise)
    Vec3 get_turbulent_wind() const;
    void set_turbulence_intensity(double intensity) { turbulence_intensity_ = intensity; }
    double get_turbulence_intensity() const { return turbulence_intensity_; }
    void enable_turbulence(bool enable) { turbulence_enabled_ = enable; }
    bool is_turbulence_enabled() const { return turbulence_enabled_; }

    // Relative velocity calculation (v_projectile - v_wind_with_turbulence)
    Vec3 get_relative_velocity(const Vec3& projectile_velocity) const;
    Vec3 get_airspeed(const Vec3& ground_speed) const;

    // Set altitude and recalculate atmospheric properties
    void set_altitude(double altitude);

    // Static methods for specific altitudes
    static double calculate_temperature(double altitude);
    static double calculate_pressure(double altitude);
    static double calculate_density(double altitude);
    static double calculate_speed_of_sound(double temperature);

private:
    double altitude_ = 0.0;        // m
    double temperature_ = SEA_LEVEL_TEMP;  // K
    double pressure_ = SEA_LEVEL_PRESSURE; // Pa
    double density_ = SEA_LEVEL_DENSITY;   // kg/m^3
    Vec3 wind_velocity_ = Vec3::zero();    // m/s

    // Turbulence parameters
    double turbulence_intensity_ = 0.1;    // 10% default turbulence (0.0 = none, 1.0 = 100%)
    bool turbulence_enabled_ = false;
    mutable std::mt19937 rng_{std::random_device{}()};  // Random number generator
    mutable std::normal_distribution<double> dist_{0.0, 1.0};  // Standard normal distribution

    void update_atmosphere();
};

} // namespace ballistx

#endif // BALLISTX_ISA_MODEL_H
