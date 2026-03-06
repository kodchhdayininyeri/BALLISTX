#include "atmosphere/isa_model.h"

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
