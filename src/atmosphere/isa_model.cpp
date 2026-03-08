#include "atmosphere/isa_model.h"
#include <cmath>

namespace ballistx {

Atmosphere::Atmosphere(double altitude)
    : altitude_(altitude) {
    update_atmosphere();
}

void Atmosphere::set_altitude(double altitude) {
    altitude_ = altitude;
    update_atmosphere();
}

void Atmosphere::set_wind(double speed_x, double speed_y, double speed_z) {
    wind_velocity_ = Vec3(speed_x, speed_y, speed_z);
}

Vec3 Atmosphere::get_turbulent_wind() const {
    if (!turbulence_enabled_ || turbulence_intensity_ <= 0.0) {
        return Vec3::zero();
    }

    // Generate Gaussian noise for each component
    double noise_x = dist_(rng_);
    double noise_y = dist_(rng_);
    double noise_z = dist_(rng_);

    // Scale turbulence by wind speed (turbulence proportional to mean wind)
    double wind_speed = wind_velocity_.magnitude();
    double turbulence_magnitude = wind_speed * turbulence_intensity_;

    // Minimum turbulence floor (1 m/s baseline for light air conditions)
    turbulence_magnitude = std::max(turbulence_magnitude, 1.0 * turbulence_intensity_);

    return Vec3(
        noise_x * turbulence_magnitude,
        noise_y * turbulence_magnitude,
        noise_z * turbulence_magnitude
    );
}

Vec3 Atmosphere::get_relative_velocity(const Vec3& projectile_velocity) const {
    // v_relative = v_projectile - v_wind_total
    // v_wind_total = v_constant_wind + v_turbulence
    Vec3 total_wind = wind_velocity_ + get_turbulent_wind();
    return projectile_velocity - total_wind;
}

Vec3 Atmosphere::get_airspeed(const Vec3& ground_speed) const {
    // Alias for get_relative_velocity - more intuitive name for aerodynamics
    return get_relative_velocity(ground_speed);
}

double Atmosphere::get_speed_of_sound() const {
    return calculate_speed_of_sound(temperature_);
}

void Atmosphere::update_atmosphere() {
    temperature_ = calculate_temperature(altitude_);
    pressure_ = calculate_pressure(altitude_);
    density_ = calculate_density(altitude_);
}

// Static implementations
double Atmosphere::calculate_temperature(double altitude) {
    if (altitude < 0.0) {
        altitude = 0.0;
    }

    if (altitude <= TROPOPAUSE_ALTITUDE) {
        // Troposphere: temperature decreases linearly
        return SEA_LEVEL_TEMP - LAPSE_RATE_TROPOSPHERE * altitude;
    } else {
        // Stratosphere (lower): constant temperature
        return STRATOSPHERE_TEMP;
    }
}

double Atmosphere::calculate_pressure(double altitude) {
    if (altitude < 0.0) {
        altitude = 0.0;
    }

    double temp = calculate_temperature(altitude);

    if (altitude <= TROPOPAUSE_ALTITUDE) {
        // Troposphere: barometric formula
        double exponent = (GRAVITY) / (GAS_CONSTANT * LAPSE_RATE_TROPOSPHERE);
        double ratio = temp / SEA_LEVEL_TEMP;
        return SEA_LEVEL_PRESSURE * std::pow(ratio, exponent);
    } else {
        // Stratosphere: exponential decay
        double tropopause_pressure = SEA_LEVEL_PRESSURE *
            std::pow(STRATOSPHERE_TEMP / SEA_LEVEL_TEMP,
                     GRAVITY / (GAS_CONSTANT * LAPSE_RATE_TROPOSPHERE));

        double delta_h = altitude - TROPOPAUSE_ALTITUDE;
        double exponent = -GRAVITY * delta_h / (GAS_CONSTANT * STRATOSPHERE_TEMP);
        return tropopause_pressure * std::exp(exponent);
    }
}

double Atmosphere::calculate_density(double altitude) {
    // Ideal gas law: rho = P / (R * T)
    double pressure = calculate_pressure(altitude);
    double temperature = calculate_temperature(altitude);
    return pressure / (GAS_CONSTANT * temperature);
}

double Atmosphere::calculate_speed_of_sound(double temperature) {
    // Speed of sound in ideal gas: a = sqrt(gamma * R * T)
    // For dry air: gamma = 1.4
    constexpr double GAMMA = 1.4;
    return std::sqrt(GAMMA * GAS_CONSTANT * temperature);
}

} // namespace ballistx
