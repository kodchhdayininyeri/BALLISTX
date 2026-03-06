#include <iostream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "utils/coriolis_effect.h"
#include "utils/vec3.h"

using namespace ballistx;
using namespace std;

/**
 * @brief Coriolis Effect Demonstration
 */
int main() {
    cout << "=======================================================\n";
    cout << "       CORIOLIS EFFECT DEMONSTRATION                \n";
    cout << "=======================================================\n\n";

    cout << "== Coriolis Effect Explained ==\n";
    cout << "Due to Earth's rotation, moving objects deflect:\n";
    cout << "- Northern hemisphere: Deflect to RIGHT\n";
    cout << "- Southern hemisphere: Deflect to LEFT\n";
    cout << "- Equator: No deflection (zero)\n";
    cout << "- Poles: Maximum deflection\n\n";

    cout << "Formula: F_coriolis = -2 * m * (Omega x v)\n\n";

    // Test 1: Deflection at different latitudes
    cout << "== Test 1: Deflection vs Latitude ==\n";
    cout << "Scenario: 155mm shell, 25km range, 80s flight time, azimuth 45°\n\n";

    double test_range = 25000.0;
    double test_time = 80.0;
    double test_azimuth = 45.0;

    cout << fixed << setprecision(2);
    for (int lat = 0; lat <= 90; lat += 15) {
        auto [deflection_east, deflection_north] = CoriolisEffect::calculate_artillery_deflection(
            test_range, test_time, lat, test_azimuth
        );

        double total_deflection = sqrt(deflection_east * deflection_east +
                                         deflection_north * deflection_north);

        cout << "Latitude " << setw(2) << lat << "°: ";
        cout << "Deflection: " << setw(6) << total_deflection << " m ("
             << setw(6) << (total_deflection * 3.28084) << " ft) - "
             << CoriolisEffect::get_deflection_direction(lat) << "\n";
    }
    cout << "\n";

    // Test 2: Sniper application
    cout << "== Test 2: Sniper Long-Range Shot ==\n";
    cout << "Scenario: 1000m shot, 45° latitude, northward\n\n";

    double sniper_range = 1000.0;
    double sniper_lat = 45.0;
    double sniper_azimuth = 0.0;  // North

    auto [moa, inches] = CoriolisEffect::calculate_sniper_correction(
        sniper_range, sniper_lat, sniper_azimuth
    );

    cout << "Range: " << sniper_range << " m (" << (sniper_range * 1.09361) << " yards)\n";
    cout << "Latitude: " << sniper_lat << "° N\n";
    cout << "Azimuth: " << sniper_azimuth << "° (North)\n\n";
    cout << "Coriolis Correction:\n";
    cout << "  " << setprecision(2) << inches << " inches ("
         << setprecision(3) << (inches * 2.54) << " cm)\n";
    cout << "  " << setprecision(2) << moa << " MOA\n\n";

    cout << "=> For precision sniping at 1000m+: MUST account for Coriolis!\n\n";

    // Test 3: Artillery scenarios
    cout << "=======================================================\n";
    cout << "       REAL-WORLD ARTILLERY EXAMPLES               \n";
    cout << "=======================================================\n\n";

    // Paris Gun (WWI) - extreme long range
    cout << "== Example 1: Paris Gun (WWI) ==\n";
    cout << "Range: 130 km, Latitude: 49° N\n";
    auto [paris_range, paris_right, paris_up] = CoriolisEffect::calculate_trajectory_deflection(
        1640.0, 55.0 * M_PI / 180.0, 49.0, 180.0
    );
    cout << "Coriolis Deflection: " << setprecision(1) << paris_right << " m RIGHT\n";
    cout << "=> At 130km range: CRITICAL for targeting!\n\n";

    // Iraqi Super Gun
    cout << "== Example 2: Iraqi Super Gun (Project Babylon) ==\n";
    cout << "Range: 600 km (planned), Latitude: 33° N\n";
    auto [gun_range, gun_right, gun_up] = CoriolisEffect::calculate_trajectory_deflection(
        2000.0, 55.0 * M_PI / 180.0, 33.0, 270.0
    );
    cout << "Estimated Deflection: " << setprecision(0) << gun_right << " m RIGHT\n";
    cout << "=> " << (gun_right / 1000.0) << " km lateral offset at 600km range!\n\n";

    // Standard 155mm howitzer
    cout << "== Example 3: Standard 155mm Howitzer ==\n";
    cout << "Range: 22 km, Latitude: 40° N, Azimuth: 90° (East)\n";
    auto [howitzer_range, howitzer_right, howitzer_up] = CoriolisEffect::calculate_trajectory_deflection(
        800.0, 45.0 * M_PI / 180.0, 40.0, 90.0
    );
    cout << "Coriolis Deflection: " << setprecision(1) << howitzer_right << " m RIGHT\n";
    cout << "=> Must compensate in fire control system\n\n";

    // Test 4: Latitude comparison
    cout << "=======================================================\n";
    cout << "       LATITUDE COMPARISON                        \n";
    cout << "=======================================================\n\n";

    cout << "Same scenario: 25km range, 80s flight time\n\n";

    struct Location {
        const char* name;
        double latitude;
    };

    Location locations[] = {
        {"Equator (0°)", 0.0},
        {"Jakarta (6°S)", -6.0},
        {"Singapore (1°N)", 1.0},
        {"Manila (14°N)", 14.0},
        {"Miami (25°N)", 25.0},
        {"Cairo (30°N)", 30.0},
        {"New Orleans (30°N)", 30.0},
        {"Houston (30°N)", 30.0},
        {"Shanghai (31°N)", 31.0},
        {"Los Angeles (34°N)", 34.0},
        {"Tokyo (35°N)", 35.0},
        {"Tehran (35°N)", 35.0},
        {"Madrid (40°N)", 40.0},
        {"Ankara (39°N)", 39.0},
        {"Beijing (40°N)", 40.0},
        {"New York (40°N)", 40.0},
        {"Rome (42°N)", 42.0},
        {"Chicago (41°N)", 41.0},
        {"Istanbul (41°N)", 41.0},
        {"London (51°N)", 51.0},
        {"Paris (49°N)", 49.0},
        {"Berlin (52°N)", 52.0},
        {"Moscow (55°N)", 55.0},
        {"Anchorage (61°N)", 61.0},
        {"North Pole (90°N)", 90.0}
    };

    cout << setw(20) << left << "Location" << " | Deflection\n";
    cout << setw(23) << setfill('-') << "-" << "+------------\n";
    cout << setfill(' ');

    for (const auto& loc : locations) {
        auto [defl_east, defl_north] = CoriolisEffect::calculate_artillery_deflection(
            25000.0, 80.0, loc.latitude, 90.0
        );
        double total_defl = sqrt(defl_east * defl_east + defl_north * defl_north);

        cout << setw(20) << left << loc.name << " | ";
        cout << setprecision(1) << setw(10) << total_defl << " m ("
             << setw(6) << (total_defl * 3.28084) << " ft)\n";
    }
    cout << "\n";

    // Test 5: Effect on different ranges
    cout << "=======================================================\n";
    cout << "       DEFLECTION VS RANGE                          \n";
    cout << "=======================================================\n\n";

    cout << "Latitude: 45° N\n\n";

    cout << fixed << setprecision(1);
    for (int range_km = 10; range_km <= 100; range_km += 10) {
        double range_m = range_km * 1000.0;
        double velocity = 800.0;
        double angle = 45.0 * M_PI / 180.0;
        double time = range_m / (velocity * std::cos(angle));

        auto [defl_east, defl_north] = CoriolisEffect::calculate_artillery_deflection(
            range_m, time, 45.0, 90.0
        );
        double total_defl = sqrt(defl_east * defl_east + defl_north * defl_north);

        double defl_pct = (total_defl / range_m) * 100.0;

        cout << "Range: " << setw(3) << range_km << " km | Deflection: "
             << setw(6) << total_defl << " m (" << setw(4) << defl_pct << "%)\n";
    }
    cout << "\n";

    // Test 6: Verification formula
    cout << "=======================================================\n";
    cout << "       PHYSICS VERIFICATION                         \n";
    cout << "=======================================================\n\n";

    double omega = CoriolisEffect::EARTH_OMEGA_MAGNITUDE;
    cout << "Earth Angular Velocity: " << (omega * 1e5) << "e-5 rad/s\n";
    cout << "Earth Rotation Period: " << CoriolisEffect::EARTH_ROTATION_PERIOD << " s\n";
    cout << "Earth Radius: " << (CoriolisEffect::EARTH_RADIUS / 1000.0) << " km\n\n";

    // Calculate Coriolis acceleration for typical shell
    Vec3 velocity(800.0, 0.0, 0.0);  // Due east
    double latitude = 45.0 * M_PI / 180.0;

    Vec3 coriolis_accel = CoriolisEffect::calculate_acceleration(velocity, latitude);
    cout << "Velocity: 800 m/s due east\n";
    cout << "Latitude: 45° N\n";
    cout << "Coriolis Acceleration: " << setprecision(6) << coriolis_accel << " m/s^2\n";
    cout << "Direction: " << ((coriolis_accel.z > 0) ? "UP" : "DOWN") << "\n";

    // Calculate deflection for 1 second
    double deflection_1s = 0.5 * coriolis_accel.magnitude() * 1.0 * 1.0;
    cout << "Deflection after 1s: " << setprecision(3) << deflection_1s << " m\n\n";

    cout << "=======================================================\n";
    cout << "       CORIOLIS DEMO COMPLETE                        \n";
    cout << "=======================================================\n";

    return 0;
}
