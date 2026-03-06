#ifndef BALLISTX_ISA_MODEL_H
#define BALLISTX_ISA_MODEL_H

#include "utils/vec3.h"
#include <cmath>

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

    // Get atmospheric properties at current altitude
    double get_temperature() const { return temperature_; }
    double get_pressure() const { return pressure_; }
    double get_density() const { return density_; }
    double get_altitude() const { return altitude_; }
    double get_speed_of_sound() const;

    // Wind
    Vec3 get_wind_velocity() const { return wind_velocity_; }
    void set_wind_velocity(const Vec3& wind) { wind_velocity_ = wind; }
    void set_wind(double speed_x, double speed_y, double speed_z);

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

    void update_atmosphere();
};

} // namespace ballistx

#endif // BALLISTX_ISA_MODEL_H
