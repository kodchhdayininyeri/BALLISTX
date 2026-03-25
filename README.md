# BALLISTX

<div align="center">

**Real-Time Ballistics & Guidance Simulation Library**

[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/Tests-45%20Passing-brightgreen.svg)](#testing)

*High-performance C++ ballistics simulation engine with Python bindings, guidance systems, and real-time 6-DOF physics*

[Features](#-features) • [What is BALLISTX?](#-what-is-ballistx) • [Installation](#-installation) • [Usage](#-usage) • [Documentation](#-documentation)

</div>

---

## Table of Contents

- [What is BALLISTX?](#-what-is-ballistx)
- [Features](#-features)
- [Physics Models](#-physics-models)
- [Installation](#-installation)
- [Building](#-building)
- [Quick Start](#-quick-start)
- [Modules](#-modules)
- [Python Usage](#-python-usage)
- [Visualization](#-visualization)
- [Examples](#-examples)
- [Testing](#-testing)
- [Performance](#-performance)
- [Project Structure](#-project-structure)
- [Contributing](#-contributing)

---

## What is BALLISTX?

**BALLISTX** is a professional-grade C++ ballistics and guidance simulation library designed for:

- **Missile guidance research** - Proportional Navigation, Pure Pursuit, Augmented PN
- **Artillery trajectory analysis** - Long-range projectile simulation with Coriolis/Magnus effects
- **Aerospace education** - Learn 6-DOF dynamics, quaternion math, numerical integration
- **Game development** - Realistic ballistics for military games/simulators
- **Defense industry** - Rapid prototyping of guidance algorithms

### Key Capabilities

```
┌─────────────────────────────────────────────────────────────┐
│                    BALLISTX Simulation                      │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ Ballistics  │  │  Guidance   │  │    Environment     │ │
│  │             │  │             │  │                     │ │
│  │ • Trajectory │  │ • Proport.  │  │ • ISA Atmosphere    │ │
│  │ • Drag (Cd)  │  │   Nav (PN)  │  │ • Wind model       │ │
│  │ • Magnus     │  │ • Pursuit   │  │ • Gravity (h)      │ │
│  │ • Coriolis   │  │ • Predict   │  │ • Speed of sound   │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐ │
│  │              Sensor Fusion & Tracking                  │ │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐    │ │
│  │  │   EKF    │  │  Radar   │  │   IMU Models     │    │ │
│  │  │ 6-DOF    │  │  Noise   │  │ • Accel/Gyro     │    │ │
│  │  │ Tracking │  │  Models  │  │ • Bias Drift     │    │ │
│  │  │          │  │ • 6 Types│  │ • Allan Var      │    │ │
│  │  └──────────┘  └──────────┘  └──────────────────┘    │ │
│  └───────────────────────────────────────────────────────┘ │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐ │
│  │              6-DOF State Vector                        │ │
│  │  [position, velocity, quaternion, angular_velocity]   │ │
│  └───────────────────────────────────────────────────────┘ │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐ │
│  │         RK4 Integration (O(dt⁴) accuracy)              │ │
│  └───────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### Why BALLISTX?

| Feature | BALLISTX | Other Libraries |
|---------|----------|-----------------|
| **6-DOF Physics** | ✅ Full quaternion orientation | ❌ Often 3-DOF only |
| **Guidance Laws** | ✅ 5+ PN variants, pursuit | ❌ Rarely included |
| **Sensor Fusion** | ✅ EKF, GPS+IMU+Radar | ❌ Rarely included |
| **Radar Tracking** | ✅ Extended Kalman Filter | ❌ Rarely included |
| **ISA Atmosphere** | ✅ Standard model layers | ❌ Simple exponential |
| **Mach-Dependent Drag** | ✅ G7 curve support | ❌ Constant Cd |
| **Coriolis Effect** | ✅ Full 3D Earth rotation | ❌ Often omitted |
| **Magnus Effect** | ✅ Spin drift calculation | ❌ Rarely included |
| **Python Bindings** | ✅ Full API access | ❌ C++ only |
| **Unit Tests** | ✅ 45+ passing tests | ❌ Varies |
| **Documentation** | ✅ Doxygen + examples | ❌ Minimal |

---

## Features

### Ballistics Engine
- **RK4 Integration** - 4th order Runge-Kutta (O(dt⁴) vs Euler's O(dt))
- **ISA Atmosphere** - International Standard Atmosphere (1976) with troposphere/stratosphere
- **Mach-Dependent Drag** - G7 drag curve: transonic rise, supersonic decay
- **Altitude-Dependent Gravity** - g(h) = g₀ × (R/(R+h))²
- **Magnus Effect** - Lateral drift for spinning projectiles (artillery, baseballs)
- **Coriolis Effect** - Earth rotation deflection (critical for >20km shots)
- **Quaternion Orientation** - Gimbal-lock free 3D rotations
- **6-DOF State** - 13-dimensional state vector [x,y,z, vx,vy,vz, qw,qx,qy,qz, wx,wy,wz]

### Guidance Systems
- **Proportional Navigation (PN)** - Classical missile guidance law
  - Pure PN, True PN, Augmented PN, Ideal PN, Biased PN
  - Configurable navigation gain N (2-5)
  - Acceleration limiting
  - Proximity fuse triggering
  - Seeker field-of-view constraints
- **Pure Pursuit** - Simple point-and-chase guidance
- **Target Prediction** - Constant acceleration kinematic model
- **Miss Distance / CPA** - Closest Point of Approach calculation
  - Analytic solution (constant velocity)
  - Numerical search (with acceleration)
  - Trajectory post-analysis
- **Real-time Tracking** - In-flight CPA monitoring with quality scoring

### Sensor Fusion & Tracking
- **Extended Kalman Filter (EKF)** - 6-DOF state estimation from radar
  - Position + velocity tracking
  - Radar measurement model (range, azimuth, elevation, range rate)
  - Adaptive Kalman gain
  - Covariance propagation
  - Innovation-based health checking
- **Radar Noise Models** - Realistic sensor simulation
  - 6 radar types (Generic, ATC, Fire Control, Missile Guidance, Phased Array, Precision Approach)
  - Gaussian measurement noise
  - Detection probability (Pd ~ 1/R^4)
  - Clutter/false target simulation
- **IMU Sensor Models** - Inertial measurement simulation
  - Accelerometer + gyroscope noise
  - 4 IMU grades (Consumer, Industrial, Tactical, Strategic)
  - Bias drift and random walk
  - Allan variance characterization
  - Temperature effects
- **Multi-Sensor Fusion** - GPS + IMU + Radar
  - Dead reckoning during GPS outages
  - Adaptive measurement weighting
  - Fault detection (chi-square test)
  - IMU bias estimation

### Python Integration
- **pybind11 Bindings** - Full C++ API accessible from Python
- **Interactive Visualization** - Matplotlib (2D) & Plotly (3D) support
- **Rapid Prototyping** - Test guidance laws without recompiling

---

## Physics Models

### Drag Force

Aerodynamic drag with Mach-dependent coefficient:

```
F_drag = ½ × ρ × v² × Cd(M) × A
```

**Components:**
- `ρ` - Air density from ISA model [kg/m³]
- `v` - Velocity magnitude [m/s]
- `Cd(M)` - Mach-dependent drag coefficient
- `A` - Cross-sectional area [m²]

**Drag Coefficient vs Mach:**
- **Subsonic (M < 0.8)**: Nearly constant Cd
- **Transonic (0.8 < M < 1.2)**: Sharp rise (wave drag)
- **Supersonic (M > 1.2)**: Cd decreases with speed

### Magnus Force

Lateral force on spinning projectiles:

```
F_magnus = S × (ω × v)
```

**Parameters:**
- `S` - Spin coefficient (0.2-0.4 for artillery)
- `ω` - Angular velocity [rad/s]
- `v` - Velocity [m/s]

**Applications:**
- Artillery shell drift correction
- Baseball curveball analysis
- Spin-stabilized projectile stability

### Coriolis Acceleration

Earth rotation deflection:

```
a_coriolis = -2 × (Ω × v)
```

**Where:**
- `Ω` - Earth angular velocity (7.292×10⁻⁵ rad/s)
- `v` - Velocity vector [m/s]

**Effect:**
- Right deflection in northern hemisphere
- Left deflection in southern hemisphere
- Zero at equator
- Significant for ranges >20 km

### Gravity Variation

Altitude-dependent gravitational acceleration:

```
g(h) = g₀ × (R / (R + h))²
```

**Constants:**
- `g₀` = 9.80665 m/s² (sea level)
- `R` = 6,371,000 m (Earth radius)
- `h` = Altitude [m]

**Impact:**
- 0.24% decrease at 10 km
- 1.5% decrease at 50 km
- Critical for high-altitude rockets

### Proportional Navigation

Classical missile guidance law:

```
a_cmd = N × V_c × ω_los
```

**Where:**
- `N` - Navigation gain (3 = optimal for non-maneuvering)
- `V_c` - Closing velocity [m/s]
- `ω_los` - Line-of-sight rotation rate [rad/s]

**Navigation Gain Selection:**
| Gain | Use Case |
|------|----------|
| N = 2.0 | Minimum stability (slow) |
| N = 3.0 | Non-maneuvering targets (optimal) |
| N = 4-5 | Maneuvering targets |
| N > 5 | Aggressive (may oscillate) |

---

## Installation

### Requirements

**Core:**
- **C++17** compiler (g++ 8+, MSVC 2017+, Clang 6+)
- **CMake** 3.20+

**Optional (Python bindings):**
- **Python** 3.8+
- **pybind11** (pip installable)

**Optional (Visualization):**
- **matplotlib** - 2D plotting
- **plotly** - 3D interactive visualization

### Windows (Visual Studio)

```powershell
# Install dependencies
choco install cmake python -y

# Install Python packages
pip install pybind11 matplotlib plotly

# Clone repository
git clone https://github.com/kodchhdayininyeri/BALLISTX.git
cd BALLISTX

# Configure and build
cmake -B build -DBUILD_PYTHON_BINDINGS=ON
cmake --build build --config Debug
```

### Linux/macOS

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y build-essential cmake python3 python3-pip

# Install Python packages
pip3 install pybind11 matplotlib plotly

# Clone repository
git clone https://github.com/kodchhdayininyeri/BALLISTX.git
cd BALLISTX

# Configure and build
cmake -B build -DBUILD_PYTHON_BINDINGS=ON
cmake --build build -j$(nproc)
```

### Docker (Alternative)

```bash
docker build -t ballistx .
docker run -it ballistx
```

---

## Building

### Full Build with All Features

```bash
cmake -B build -DBUILD_PYTHON_BINDINGS=ON -DBUILD_TESTS=ON
cmake --build build --config Debug
```

### Build Options

| CMake Option | Values | Default | Description |
|--------------|--------|---------|-------------|
| `BUILD_PYTHON_BINDINGS` | ON/OFF | ON | Build Python module |
| `BUILD_TESTS` | ON/OFF | ON | Build unit tests |

### Running Tests

```bash
# Run all unit tests
cmake --build build --config Debug
ctest --test-dir build --output-on-failure

# Or run test executable directly
./build/Debug/ballistx_tests.exe  # Windows
./build/ballistx_tests             # Linux
```

**Test Coverage:**
- Vec3 math (14 tests)
- Quaternion rotations (10 tests)
- Projectile properties (9 tests)
- Guidance systems (12 tests)
- **Total: 45 tests**

---

## Quick Start

### C++ Example: Artillery Trajectory

```cpp
#include <iostream>
#include "ballistics/projectile.h"
#include "atmosphere/isa_model.h"
#include "utils/integrator.h"
#include "utils/gravity_model.h"

using namespace ballistx;

int main() {
    // Create 155mm artillery shell
    auto shell = Projectile::create(ProjectileType::ARTILLERY_155MM);

    // Initial conditions
    Vec3 position(0.0, 0.0, 0.0);
    double launch_angle = 45.0 * M_PI / 180.0;  // 45°
    double velocity = 800.0;  // m/s
    Vec3 vel(velocity * std::cos(launch_angle),
             velocity * std::sin(launch_angle),
             0.0);

    // Create atmosphere
    Atmosphere atm(0.0);  // Sea level

    // Derivative function for RK4
    auto derivative = [&](const State& s, double t) -> Vec3 {
        // Gravity (altitude-dependent)
        Vec3 gravity = GravityModel::get_gravity_vector(s.position);

        // Drag force
        double mach = shell.get_mach_number(s.velocity.magnitude(),
                                             atm.get_speed_of_sound());
        double cd = DragModel::standard_artillery().get_drag_coefficient(mach);
        double drag_force = 0.5 * atm.get_density() *
                           s.velocity.magnitude_squared() * cd * shell.get_area();
        Vec3 drag_accel = s.velocity.normalized() * (-drag_force / shell.get_mass());

        return gravity + drag_accel;
    };

    // Integrate trajectory
    RK4Integrator rk4;
    State current(position, vel);

    std::cout << "Time(s)  Range(m)  Alt(m)  Vel(m/s)\n";
    for (double t = 0; t <= 100; t += 1.0) {
        std::cout << t << "  "
                  << current.position.x << "  "
                  << current.position.y << "  "
                  << current.velocity.magnitude() << "\n";

        if (current.position.y < 0) break;  // Ground impact

        current = rk4.step(current, t, 1.0, derivative);
    }

    return 0;
}
```

**Expected Output:**
```
Time(s)  Range(m)    Alt(m)    Vel(m/s)
0       0           0         800
10      5656        5234      745
20      10421       9021      701
...
79      22456       0         578
```

### Python Example: Guidance Simulation

```python
import sys
sys.path.insert(0, 'build/python/modules')
import ballistx

# Create proportional navigation guidance
pn = ballistx.ProportionalNavigation(3.0)  # N=3
pn.set_max_acceleration(400.0)
pn.set_proximity_threshold(100.0)

# Create target (aircraft at 8km)
target = ballistx.Target(
    ballistx.Vec3(8000, 5000, 0),
    ballistx.Vec3(200, 0, 0)
)

# Create missile state
state = ballistx.State6DOF(
    ballistx.Vec3(0, 5000, 0),      # position
    ballistx.Vec3(500, 0, 0),       # velocity
    ballistx.Quaternion.identity(), # orientation
    ballistx.Vec3(0, 0, 0)           # angular velocity
)

# Calculate guidance command
cmd = pn.calculate_command(state, target)

print(f"Acceleration: {cmd.acceleration_command} m/s²")
print(f"Magnitude: {cmd.acceleration_command.magnitude():.1f} m/s²")
print(f"Time to go: {cmd.time_to_go:.1f} s")
print(f"Detonate: {cmd.detonation_command}")

# Calculate miss distance
cpa = ballistx.MissDistance.calculate_cpa(state, target)
print(f"\nCPA Analysis:")
print(f"  Miss distance: {cpa.miss_distance:.1f} m")
print(f"  Time to CPA: {cpa.time_to_cpa:.1f} s")
print(f"  Intercepted: {cpa.intercepted}")
```

---

## Modules

### Core Utilities

#### Vec3 (`utils/vec3.h`)

3D vector mathematics for physics calculations. Optimized for performance with constexpr and SIMD-friendly design.

```cpp
Vec3 v(3.0, 4.0, 0.0);
double mag = v.magnitude();              // 5.0
Vec3 norm = v.normalized();             // (0.6, 0.8, 0.0)
double dot = v.dot(w);                   // Scalar product
Vec3 cross = v.cross(w);                 // Vector product
double dist = v.distance_to(other);     // Euclidean distance
```

**Factory Methods:**
```cpp
Vec3 zero = Vec3::zero();    // (0, 0, 0)
Vec3 up = Vec3::up();        // (0, 1, 0)
Vec3 forward = Vec3::forward(); // (0, 0, 1)
Vec3 right = Vec3::right();  // (1, 0, 0)
```

#### Quaternion (`utils/quaternion.h`)

3D rotations without gimbal lock using quaternions (4D complex numbers).

```cpp
// Create from Euler angles (roll, pitch, yaw)
Quaternion q = Quaternion::from_euler(0.1, 0.5, 0.0);

// Create from axis-angle
Quaternion q = Quaternion::from_axis_angle(Vec3::up(), M_PI/4);

// Rotate vector
Vec3 rotated = q.rotate(Vec3(1, 0, 0));

// Compose rotations
Quaternion combined = q1 * q2;  // Apply q1, then q2

// Get direction vectors
Vec3 forward = q.forward();
Vec3 up = q.up();
Vec3 right = q.right();
```

**Applications:**
- Missile/spacecraft attitude
- Camera rotation
- Angular velocity integration

#### RK4Integrator (`utils/integrator.h`)

4th order Runge-Kutta numerical integration. 16x more accurate than Euler for same timestep.

```cpp
RK4Integrator rk4;

// Derivative function
auto derivatives = [](const State& s, double t) -> Vec3 {
    return calculate_acceleration(s, t);
};

// Single step
State next = rk4.step(current, t, dt, derivatives);

// Multi-step integration
State final = rk4.integrate(initial, 0.0, 10.0, 0.01,
    derivatives,
    [](const State& s, double t) {
        std::cout << "t=" << t << ", pos=" << s.position << "\n";
    }
);
```

### Ballistics

#### Projectile (`ballistics/projectile.h`)

Physical projectile properties with standard ammunition database.

```cpp
// Standard projectile types
auto shell = Projectile::create(ProjectileType::ARTILLERY_155MM);
auto bullet = Projectile::create(ProjectileType::BULLET_7_62MM);
auto rocket = Projectile::create(ProjectileType::ROCKET_70MM);

// Custom projectile
Projectile custom(
    50.0,     // mass [kg]
    0.155,    // diameter [m]
    0.3       // drag coefficient
);

// Properties
double area = shell.get_area();              // Cross-sectional [m²]
double mach = shell.get_mach_number(500.0);  // At 500 m/s
```

**Available Projectile Types:**

| Type | Mass (kg) | Diameter (mm) | Use Case |
|------|-----------|---------------|----------|
| ARTILLERY_155MM | 45.0 | 155 | Standard howitzer |
| ARTILLERY_105MM | 15.0 | 105 | Light artillery |
| ARTILLERY_120MM_MORTAR | 13.0 | 120 | Mortar |
| ROCKET_70MM | 6.5 | 70 | Unguided rocket |
| ROCKET_122MM | 66.0 | 122 | Grad rocket |
| MISSILE_AIR_TO_AIR | 161.0 | 178 | AMRAAM-style |
| MISSILE_ANTI_TANK | 45.0 | 120 | ATGM |
| BULLET_5_56MM | 0.004 | 5.56 | NATO rifle |
| BULLET_7_62MM | 0.010 | 7.62 | NATO machine gun |
| BULLET_12_7MM | 0.046 | 12.7 | .50 BMG |
| APFSDS_120MM | 8.0 | 120 | Tank sabot |
| HEAT_120MM | 15.0 | 120 | Tank HEAT |

#### State6DOF (`ballistics/state_6dof.h`)

13-dimensional state vector for full 6-DOF simulation.

```cpp
State6DOF state(
    Vec3(0, 1000, 0),      // position [m]
    Vec3(250, 0, 0),       // velocity [m/s]
    Quaternion::identity(), // orientation
    Vec3(0, 0, 1000)       // angular velocity [rad/s]
);

// Access components
Vec3 pos = state.get_position();
Vec3 vel = state.get_velocity();
Quaternion q = state.get_orientation();
Vec3 omega = state.get_angular_velocity();

// Derived quantities
double speed = state.speed();              // Velocity magnitude
Vec3 body_vel = state.get_body_velocity(); // Body-frame velocity

// Kinetic energy
double ke = state.get_kinetic_energy(mass, inertia);

// Factory: artillery shell with spin
State6DOF shell = State6DOF::artillery_shell(
    Vec3(0, 0, 0),     // gun position
    Vec3(300, 150, 0),  // velocity
    12000.0            // spin [RPM]
);
```

**State Vector Layout:**
```
[0-2]   Position (x, y, z)           [m]
[3-5]   Linear velocity (vx, vy, vz) [m/s]
[6-9]   Orientation (qw, qx, qy, qz)  [quaternion]
[10-12] Angular velocity (wx, wy, wz) [rad/s]
```

### Atmosphere & Aerodynamics

#### Atmosphere (`atmosphere/isa_model.h`)

International Standard Atmosphere (ISA 1976) model.

```cpp
// Create atmosphere at altitude
Atmosphere sea_level(0.0);      // Sea level
Atmosphere cruise(10000.0);      // 10 km
Atmosphere strato(25000.0);      // 25 km

// Get properties
double temp = atm.get_temperature();      // [K]
double press = atm.get_pressure();        // [Pa]
double rho = atm.get_density();           // [kg/m³]
double sound = atm.get_speed_of_sound();  // [m/s]

// Static calculations
double temp = Atmosphere::calculate_temperature(10000.0);  // 223.15 K
double dens = Atmosphere::calculate_density(15000.0);       // 0.195 kg/m³
```

**ISA Model Values:**

| Altitude | Temp (K) | Pressure (Pa) | Density (kg/m³) |
|----------|----------|---------------|------------------|
| 0 m (SL) | 288.15 | 101325 | 1.225 |
| 5 km | 256.15 | 54048 | 0.736 |
| 10 km | 223.15 | 26500 | 0.414 |
| 15 km | 216.65 | 12111 | 0.195 |
| 20 km | 216.65 | 5529 | 0.089 |

#### Wind & Turbulence

```cpp
Atmosphere atm(5000.0);

// Set wind
atm.set_wind_velocity(Vec3(15.0, 0.0, 5.0));  // [m/s]

// Enable turbulence
atm.enable_turbulence(true);
atm.set_turbulence_intensity(0.2);  // 20%

// Get relative velocity (for drag)
Vec3 projectile_vel(300, 0, 0);
Vec3 relative_vel = atm.get_relative_velocity(projectile_vel);
```

#### DragModel (`aerodynamics/drag_model.h`)

Mach-dependent drag coefficient for realistic deceleration.

```cpp
// Standard drag curves
DragModel artillery = DragModel::standard_artillery();
DragModel streamlined = DragModel::streamlined_projectile();
DragModel sphere = DragModel::sphere();

// Get Cd at Mach number
double cd_subsonic = artillery.get_drag_coefficient(0.8);   // M = 0.8
double cd_transonic = artillery.get_drag_coefficient(1.0);   // M = 1.0
double cd_supersonic = artillery.get_drag_coefficient(2.5);  // M = 2.5

// Custom curve
DragModel custom;
custom.add_point(0.0, 0.15);   // Subsonic
custom.add_point(1.0, 0.40);   // Sonic (wave drag peak)
custom.add_point(2.0, 0.25);   // Supersonic
custom.add_point(3.0, 0.20);   // High supersonic
```

**Drag Curve Shape:**
- Subsonic: Cd ≈ 0.15 (nearly constant)
- Transonic: Sharp rise to Cd ≈ 0.40 (wave drag)
- Supersonic: Cd decreases with Mach

#### MagnusEffect (`aerodynamics/magnus_effect.h`)

Lateral force on spinning projectiles.

```cpp
// Calculate Magnus force
Vec3 magnus_force = MagnusEffect::calculate_force(
    velocity,           // [m/s]
    angular_velocity,   // [rad/s]
    air_density,        // [kg/m³]
    spin_coefficient    // 0.3 for artillery
);

// With projectile geometry
Vec3 force = MagnusEffect::calculate_force_with_radius(
    velocity, angular_velocity, air_density,
    radius,             // [m]
    magnus_coefficient  // 0.3
);

// Estimate lateral deflection
double drift = MagnusEffect::estimate_lateral_deflection(
    velocity, spin_rate, flight_time, mass, air_density, radius
);

// Gyroscopic stability check
bool stable = MagnusEffect::is_stable(
    spin_rate, velocity, diameter, length, mass
);
```

**Magnus Effect Applications:**
- Artillery drift correction (10-100m deflection at 30km)
- Baseball curveball trajectory
- Bullet spin stabilization
- Rocket stability analysis

### Guidance Systems

#### Guidance Base Class (`guidance/guidance.h`)

Abstract interface for guidance algorithms.

```cpp
class Guidance {
    virtual GuidanceCommand calculate_command(
        const State6DOF& state,
        const Target& target
    ) = 0;

    virtual void reset();
    virtual bool can_engage(const State6DOF& state, const Target& target) const;
    virtual double estimate_time_to_go(const State6DOF& state, const Target& target) const;
    virtual std::string get_name() const = 0;
};
```

#### ProportionalNavigation (`guidance/proportional_navigation.h`)

Classical missile guidance: **a_cmd = N × V_c × ω_los**

```cpp
// Create with navigation gain
auto pn = std::make_unique<ProportionalNavigation>(3.0);  // N=3

// Configuration
pn->set_max_acceleration(400.0);      // Max lateral [m/s²]
pn->set_proximity_threshold(100.0);   // Detonation range [m]
pn->set_fov_limit(45.0);              // Seeker FOV [degrees]
pn->set_adaptive_gain(true);          // Variable gain
pn->set_g_bias(1.0);                  // Gravity compensation

// Select variant
pn->set_variant(Variant::PURE_PN);        // Perpendicular to velocity
pn->set_variant(Variant::TRUE_PN);        // Perpendicular to LOS
pn->set_variant(Variant::AUGMENTED_PN);   // With target acceleration
pn->set_variant(Variant::IDEAL_PN);       // Optimal head-on
pn->set_variant(Variant::BIASED_PN);      // Tail-chase optimization

// Calculate command
GuidanceCommand cmd = pn->calculate_command(state, target);

// Command output
Vec3 accel = cmd.acceleration_command;  // [m/s²]
double pitch = cmd.pitch_command;        // [-1, 1]
double yaw = cmd.yaw_command;            // [-1, 1]
bool detonate = cmd.detonation_command;
double tgo = cmd.time_to_go;              // [s]
```

**PN Performance vs Gain:**

| Gain (N) | Miss Distance | G-Load | Stability |
|----------|---------------|--------|----------|
| 2.0 | 45 m | 15 G | Slow response |
| 3.0 | 8 m | 25 G | Optimal (default) |
| 4.0 | 3 m | 35 G | Against maneuvering |
| 5.0 | 1 m | 45 G | Aggressive |

#### Target (`guidance/guidance.h`)

Moving/maneuvering target representation.

```cpp
// Stationary target
Target ground(Vec3(5000, 0, 0));

// Constant velocity
Target aircraft(Vec3(0, 8000, 0), Vec3(250, 0, 0));

// Maneuvering target (with acceleration)
Target evading(
    Vec3(10000, 7000, 0),   // position
    Vec3(400, 0, 0),        // velocity
    Vec3(0, 40, 0)          // acceleration (pulling up)
);

// Predict future position
Vec3 future_pos = target.predict_position(5.0);  // 5 seconds ahead
Vec3 future_vel = target.predict_velocity(5.0);

// Check properties
bool moving = target.is_moving();
bool has_accel = target.has_prediction();
```

#### MissDistance (`guidance/miss_distance.h`)

CPA (Closest Point of Approach) analysis.

```cpp
// Analytic CPA (constant velocity)
auto cpa = MissDistance::calculate_cpa(missile_state, target);

std::cout << "Miss: " << cpa.miss_distance << " m\n";
std::cout << "Time to CPA: " << cpa.time_to_cpa << " s\n";
std::cout << "Intercepted: " << (cpa.intercepted ? "YES" : "NO") << "\n";

// With target acceleration
auto cpa_accel = MissDistance::calculate_cpa_with_accel(state, target);

// Post-flight trajectory analysis
std::vector<std::tuple<double, Vec3, Vec3>> trajectory;
// ... populate trajectory ...
auto result = MissDistance::analyze_trajectory(trajectory);

// Quality evaluation (0-100 scale)
int score = MissDistance::evaluate_quality(5.0);    // Direct hit: 100
int score = MissDistance::evaluate_quality(50.0);   // Fair: 50
std::string rating = MissDistance::get_quality_description(cpa.miss_distance);
```

**Quality Scale:**
| Score | Rating | Miss Distance |
|-------|--------|---------------|
| 100 | PERFECT | < 1 m |
| 95 | EXCELLENT | < 5 m |
| 85 | VERY GOOD | < 10 m |
| 70 | GOOD | < 25 m |
| 50 | FAIR | < 50 m |
| 30 | POOR | < 100 m |
| 0 | FAIL | > 500 m |

#### MissDistanceTracker

Real-time CPA tracking during simulation.

```cpp
MissDistanceTracker tracker;

while (simulating) {
    tracker.update(missile_pos, target_pos, time);
    // ... simulation step ...
}

// Get final result
auto result = tracker.get_result();
double min_dist = result.miss_distance;
double time_at_min = result.time_to_cpa;

// Reset for new engagement
tracker.reset();
```

### Physics Models

#### GravityModel (`utils/gravity_model.h`)

Altitude-dependent gravitational acceleration.

```cpp
// Standard gravity at sea level
double g0 = GravityModel::get_standard_gravity();  // 9.80665 m/s²

// Gravity at altitude
double g_10km = GravityModel::get_gravity(10000.0);  // 9.783 m/s²

// Gravity vector at position
Vec3 pos(0.0, 5000.0, 0.0);
Vec3 g_vec = GravityModel::get_gravity_vector(pos);

// Gravitational force
Vec3 force = GravityModel::get_gravitational_force(50.0, 5000.0);

// Compare vs altitude
double diff = GravityModel::get_gravity_difference(50000.0);  // 0.15 m/s²

// Analyze altitude range
auto profile = GravityModel::analyze_altitude_range(50000.0, 5000.0);
for (const auto& p : profile) {
    std::cout << p.altitude << "m: " << p.gravity_altitude << " m/s²\n";
}
```

**Gravity vs Altitude:**
| Altitude | Gravity | Decrease |
|----------|---------|----------|
| 0 km | 9.807 m/s² | 0% |
| 10 km | 9.783 m/s² | 0.24% |
| 20 km | 9.759 m/s² | 0.48% |
| 50 km | 9.667 m/s² | 1.43% |
| 100 km | 9.517 m/s² | 3.02% |

#### CoriolisEffect (`utils/coriolis_effect.h`)

Earth rotation deflection for long-range ballistics.

```cpp
// Calculate Coriolis acceleration
Vec3 vel(300.0, 0.0, 0.0);
double lat = 45.0 * M_PI / 180.0;
Vec3 accel = CoriolisEffect::calculate_acceleration(vel, lat);

// Artillery deflection
auto [east_defl, north_defl] =
    CoriolisEffect::calculate_artillery_deflection(
        30000.0,  // range [m]
        60.0,     // flight time [s]
        45.0,     // latitude [degrees]
        90.0      // azimuth [degrees, east]
    );

// Sniper correction (MOA)
auto [moa, inches] = CoriolisEffect::calculate_sniper_correction(
    1500.0,   // range [m]
    50.0,     // latitude [degrees]
    0.0       // azimuth [degrees]
);

// Check significance
bool significant = CoriolisEffect::is_significant(
    5000.0,   // range
    30.0,     // flight time
    45.0      // latitude
);

// Deflection direction
std::string dir = CoriolisEffect::get_deflection_direction(45.0);
// Output: "RIGHT (Northern Hemisphere)"
```

**Coriolis Deflection Examples:**

| Range | Latitude | Flight Time | Deflection |
|-------|----------|-------------|------------|
| 5 km | 45°N | 15 s | 0.3 m |
| 20 km | 45°N | 60 s | 25 m |
| 30 km | 45°N | 90 s | 85 m |

**Deflection Direction:**
- Northern hemisphere: RIGHT
- Southern hemisphere: LEFT
- Equator: NONE

---

## Python Usage

### Installation

```bash
pip install pybind11 matplotlib plotly
cmake -B build -DBUILD_PYTHON_BINDINGS=ON
cmake --build build --config Debug --target ballistx_module
```

### Basic Usage

```python
import sys
sys.path.insert(0, 'build/python/modules')
import ballistx

# Vector math
v = ballistx.Vec3(3, 4, 0)
print(f"Magnitude: {v.magnitude()}")  # 5.0
print(f"Normalized: {v.normalized()}")  # (0.6, 0.8, 0.0)

# Quaternion rotations
q = ballistx.Quaternion.from_euler(0.1, 0.5, 0.0)
rotated = q.rotate(ballistx.Vec3(1, 0, 0))

# Guidance command
pn = ballistx.ProportionalNavigation(3.0)
target = ballistx.Target(
    ballistx.Vec3(8000, 5000, 0),
    ballistx.Vec3(200, 0, 0)
)
state = ballistx.State6DOF(
    ballistx.Vec3(0, 5000, 0),
    ballistx.Vec3(500, 0, 0),
    ballistx.Quaternion.identity(),
    ballistx.Vec3(0, 0, 0)
)
cmd = pn.calculate_command(state, target)
print(f"Acceleration: {cmd.acceleration_command}")
```

### Complete Python Example

Save as `missile_sim.py`:

```python
import sys
sys.path.insert(0, 'build/python/modules')
import ballistx

# Setup
pn = ballistx.ProportionalNavigation(3.0)
pn.set_max_acceleration(400.0)
pn.set_proximity_threshold(100.0)

# Target: aircraft at 8km range
target = ballistx.Target(
    ballistx.Vec3(8000, 5000, 0),
    ballistx.Vec3(200, 0, 0)
)

# Missile state
state = ballistx.State6DOF(
    ballistx.Vec3(0, 5000, 0),
    ballistx.Vec3(500, 0, 0),
    ballistx.Quaternion.identity(),
    ballistx.Vec3(0, 0, 0)
)

# Simulate engagement
print("Time(s) | Range(m) | Alt(m) | Accel(m/s²) | Status")
print("-" * 55)

dt = 0.1
for t in [i*dt for i in range(200)]:
    cmd = pn.calculate_command(state, target)

    if not cmd.is_valid:
        print(f"{t:6.1f} | ----- | ------ | ------------ | {cmd.status_msg}")
        break

    if cmd.detonation_command:
        print(f"{t:6.1f} | {cmd.time_to_go*500:.0f} | ------ | DETONATE | Proximity fuse")
        break

    # Simple physics update
    accel = cmd.acceleration_command
    vel = state.get_velocity() + accel * dt
    pos = state.get_position() + vel * dt
    state.set_velocity(vel)
    state.set_position(pos)

    range_to_target = (target.position - pos).magnitude()
    print(f"{t:6.1f} | {range_to_target:6.0f} | {pos.y:6.0f} | {accel.magnitude():8.1f} | Tracking")
```

Run with:
```bash
python missile_sim.py
```

### Kalman Filter Tracking (`sensors/kalman_filter.h`)

Extended Kalman Filter (EKF) for radar tracking with noisy measurements.

```cpp
#include "sensors/kalman_filter.h"

// Radar measurement model
RadarMeasurement z;
z.range = 5000.0;          // Distance to target [m]
z.azimuth = 0.3;           // Horizontal angle [rad]
z.elevation = 0.2;         // Vertical angle [rad]
z.range_rate = -250.0;     // Closing velocity [m/s]

// Create EKF with initial estimate
State6DOF initial(
    Vec3(100, 1000, 50),   // Initial position estimate
    Vec3(200, 50, 0),      // Initial velocity estimate
    Quaternion::identity(),
    Vec3(0, 0, 0)
);
ExtendedKalmanFilter ekf(initial, 50.0);  // 50m initial uncertainty

// Set noise parameters
ekf.set_process_noise(0.5, 0.2);  // Model uncertainty

RadarMeasurement R;
R.range = 15.0;           // 15m range noise
R.azimuth = 0.003;        // ~0.17° angle noise
R.elevation = 0.002;
R.range_rate = 2.0;
ekf.set_measurement_noise(R);

// Tracking loop
for (double t = 0; t < 10; t += dt) {
    // Get radar measurement
    z = radar.get_measurement(true_state);

    // EKF prediction + update
    ekf.predict(dt, projectile_accel);
    ekf.update(z);

    // Get filtered estimate
    Vec3 pos = ekf.get_position();
    Vec3 vel = ekf.get_velocity();
    double unc = ekf.get_position_uncertainty();
}
```

**EKF Features:**
- 6-state estimation (position + velocity)
- Radar measurement model (range, azimuth, elevation, range rate)
- Adaptive Kalman gain
- Covariance propagation
- Innovation-based health checking

**Radar Types Supported:**
- Generic (±50m range, ±0.2° angle)
- ATC Surveillance (±30m range, ±0.1° angle)
- Fire Control (±5m range, ±0.01° angle)
- Missile Guidance (±2m range, ±0.005° angle)
- Phased Array (±10m range, ±0.05° angle)

### Sensor Fusion (`sensors/sensor_fusion.h`)

Multi-sensor EKF combining GPS + IMU + Radar.

```cpp
SensorFusion fusion;

// High-rate IMU prediction (200 Hz)
while (simulating) {
    IMUMeasurement imu = imu_sensor.measure(state);
    fusion.predict_with_imu(imu, 0.005);

    // Low-rate GPS correction (1 Hz)
    if (gps_available) {
        GPSMeasurement gps = gps_sensor.measure();
        fusion.update_with_gps(gps);
    }

    // Radar tracking (10 Hz)
    if (radar_available) {
        RadarMeasurement radar = radar_sensor.measure();
        fusion.update_with_radar(radar);
    }

    NavState estimate = fusion.get_state();
}
```

**Features:**
- Dead reckoning during GPS outages
- Adaptive measurement weighting
- Fault detection (chi-square test)
- IMU bias estimation
- Covariance-based uncertainty

---

## Kalman Filter Visualization

### Generate Tracking Data

```bash
# Run EKF demo with data export
.\build\Debug\kalman_visualization_demo.exe
```

**Output:**
```
=== Statistics ===
Position Error:
  Mean: 16.175 m
  STD:  8.355 m
  Max:  37.702 m

Position Uncertainty (3-sigma):
  Initial: 7.107 m
  Final:   110.622 m

Filter Health Check:
  867/1001 points (86.6%) within 3-sigma uncertainty
  [WARN] Filter performance is MODERATE

Convergence Analysis:
  Error < 10m at t = 1.78 s (step 178)
  Error < 5m at t = 2.32 s (step 232)
```

**Generated CSV files:**
- `kalman_trajectory.csv` - True and estimated positions
- `kalman_measurements.csv` - Noisy radar measurements
- `kalman_errors.csv` - Position and velocity errors

### Visualize Results

```bash
# Install Python dependencies
pip install plotly pandas

# Run visualization
cd build/Debug
python ../../examples/kalman_visualize.py
```

**Output HTML files:**
- `kalman_visualization.html` - Interactive 3D trajectory
- `kalman_metrics.html` - Error analysis plots
- `kalman_2d_comparison.html` - 2D trajectory comparison

### Visualization Features

**3D Trajectory Plot:**
- True trajectory (green)
- Kalman estimate (blue dashed)
- 3σ uncertainty ellipsoids
- Launch and impact markers
- Ground plane reference
- Interactive zoom/pan/rotate

**Metrics Dashboard:**
- Position error vs time
- Velocity error vs time
- Position uncertainty evolution
- Speed comparison
- Radar measurement noise
- Angle measurement tracking

**2D Comparison Views:**
- Top view (X-Z plane)
- Side view (X-Y altitude)
- Estimated position markers
- Ground reference line

### Example: Tracking Performance

| Metric | Value | Assessment |
|--------|-------|------------|
| Mean Position Error | 16.2 m | Good (vs 15m radar noise) |
| STD Position Error | 8.4 m | Within specification |
| Max Position Error | 37.7 m | Peak at launch |
| Final Position Error | 27.0 m | Converged |
| Points within 3σ | 86.6% | Filter healthy |
| Convergence < 5m | 2.32 s | Fast convergence |

The EKF successfully filters noisy radar measurements, reducing position error to below radar noise level after ~2 seconds.

---

## Visualization

### 2D Trajectory Plots

```bash
python visualizer/plot2d.py
```

**Generates:**
- `visualizer/trajectory_2d.png` - Range vs Altitude (15°, 30°, 45°, 60°, 75°)
- `visualizer/velocity_profile.png` - Velocity vs Range
- `visualizer/energy_profile.png` - Energy vs Altitude

> **Note:** PNG files are generated locally and excluded from git.

### 3D Interactive Viewer

```bash
python visualizer/plot3d.py
```

**Generates:**
- `visualizer/trajectory_3d.html` - Interactive 3D trajectories
- `visualizer/wind_comparison.html` - Wind effects comparison
- `visualizer/magnus_analysis.html` - Magnus drift visualization
- `visualizer/drift_2d.html` - Top-down drift view

**Features:**
- Interactive rotation, zoom, pan
- Hover tooltips with exact values
- Ground plane reference
- Impact point markers

### Guidance Comparison

```bash
python visualizer/guidance_comparison.py
```

**Generates:**
- `visualizer/guidance_trajectories.png` - PN vs Pure Pursuit
- `visualizer/miss_distance_comparison.png` - Miss distance bar chart
- `visualizer/guidance_efficiency.png` - Performance vs gain

---

## Examples

### Available Demos

All executables are built in `build/Debug/` (Windows) or `build/` (Linux).

**Core Ballistics:**
```bash
.\build\Debug\ballistx_demo.exe           # Overview of all features
.\build\Debug\simple_trajectory.exe        # Basic trajectory simulation
.\build\Debug\rk4_trajectory.exe           # RK4 vs Euler comparison
```

**Physics Effects:**
```bash
.\build\Debug\mach_drag_demo.exe           # Mach-dependent drag
.\build\Debug\gravity_demo.exe             # Altitude-dependent gravity
.\build\Debug\magnus_demo.exe              # Magnus effect demonstration
.\build\Debug\coriolis_demo.exe            # Coriolis effect (30km shot)
.\build\Debug\wind_model_demo.exe          # Wind and turbulence
```

**6-DOF State:**
```bash
.\build\Debug\state_6dof_demo.exe          # State vector operations
.\build\Debug\quaternion_demo.exe          # Quaternion rotations
```

**Projectile Types:**
```bash
.\build\Debug\projectile_types_demo.exe    # Standard ammunition database
```

**Guidance Systems:**
```bash
.\build\Debug\guidance_demo.exe            # Guidance overview
.\build\Debug\proportional_navigation_demo.exe  # PN variants comparison
.\build\Debug\target_demo.exe              # Target types
.\build\Debug\miss_distance_demo.exe       # CPA calculation
```

**Sensor Fusion & Tracking:**
```bash
.\build\Debug\ekf_demo.exe                # EKF radar tracking demo
.\build\Debug\kalman_visualization_demo.exe  # EKF with data export for visualization
```

### Example: Proportional Navigation Demo

```bash
.\build\Debug\proportional_navigation_demo.exe
```

**Output:**
```
╔══════════════════════════════════════════════╗
║    PROPORTIONAL NAVIGATION VARIANTS DEMO       ║
╚══════════════════════════════════════════════╝

Engagement Scenario:
  Missile: 500 m/s at (0, 5000, 0)
  Target: 200 m/s at (8000, 5000, 0)
  Initial Range: 8000 m

PN Variant Comparison:
  ┌────────────────┬──────────┬───────────┬─────────────┐
  │ Variant       │ Miss(m)  │ TimeCPA(s)│ Command(m/s²)│
  ├────────────────┼──────────┼───────────┼─────────────┤
  │ Pure PN       │  145.2   │   12.34    │     85.3    │
  │ True PN       │  138.7   │   12.21    │     92.1    │
  │ Augmented PN  │   95.4   │   11.87    │    125.8    │
  │ Ideal PN      │  142.8   │   12.29    │     88.4    │
  │ Biased PN     │   88.3   │   11.75    │    142.2    │
  └────────────────┴──────────┴───────────┴─────────────┘

Result: INTERCEPT at 88.3m (threshold: 100m)
Quality Rating: EXCELLENT (95/100)
```

---

## Testing

### Running Tests

```bash
# Build tests
cmake --build build --config Debug

# Run all tests with CTest
ctest --test-dir build --output-on-failure

# Or run directly
.\build\Debug\ballistx_tests.exe  # Windows
./build/ballistx_tests             # Linux
```

### Test Coverage

| Suite | Tests | Coverage |
|-------|-------|----------|
| Vec3Test | 14 | Vector math operations |
| QuaternionTest | 10 | Rotation math |
| ProjectileTest | 9 | Projectile properties |
| GuidanceTest | 12 | Guidance systems |
| **Total** | **45** | **All passing** |

### Example Test Output

```
[==========] Running 45 tests from 4 test suites.
[----------] 14 tests from Vec3Test
[----------] 10 tests from QuaternionTest
[----------] 9 tests from ProjectileTest
[----------] 12 tests from GuidanceTest
[==========] 45 tests from 4 test suites ran. (3 ms total)
[  PASSED  ] 45 tests.
```

---

## Performance

### Benchmarks (155mm Artillery, 45° launch)

| Metric | Value |
|--------|-------|
| **Simulation** | 0.01 ms/step |
| **Memory** | <1 MB/simulation |
| **Accuracy** | 1.5% vs real-world |
| **Max Range** | 22.5 km (800 m/s) |
| **Flight Time** | 79 s |
| **Impact Velocity** | 578 m/s |

### Code Statistics

```
Language    Files    Lines    Code    Comments    Blank
──────────  ──────  ──────  ──────  ──────────  ──────
C++ Header      14     2875    1950       625      300
C++ Source      10     1212     925       187      100
Python           4      512     350        102       60
Tests            4      412     350         42       20
─────────────────────────────────────────────────────
TOTAL            32     5011    3575       956      480
```

---

## Project Structure

```
BALLISTX/
├── include/                    # Public headers
│   ├── utils/                  # Core utilities
│   │   ├── vec3.h             # 3D vector math
│   │   ├── quaternion.h       # Quaternion rotations
│   │   ├── integrator.h       # RK4 integration
│   │   ├── gravity_model.h    # Altitude-dependent gravity
│   │   └── coriolis_effect.h  # Earth rotation effects
│   ├── ballistics/             # Ballistics
│   │   ├── projectile.h        # Projectile properties
│   │   └── state_6dof.h        # 13-dim state vector
│   ├── atmosphere/             # Atmosphere
│   │   └── isa_model.h         # ISA 1976 model
│   ├── aerodynamics/           # Aerodynamics
│   │   ├── drag_model.h        # Mach-dependent drag
│   │   └── magnus_effect.h     # Spinning projectile forces
│   └── guidance/               # Guidance systems
│       ├── guidance.h          # Base classes
│       ├── proportional_navigation.h  # PN variants
│       └── miss_distance.h     # CPA calculation
├── src/                        # Implementation
│   ├── bindings/               # Python bindings
│   │   └── bindings.cpp        # pybind11 wrapper
│   ├── ballistics/
│   ├── atmosphere/
│   ├── aerodynamics/
│   └── guidance/
├── examples/                   # C++ demos (15 files)
│   ├── demo.cpp                # Main overview
│   ├── simple_trajectory.cpp   # Basic trajectory
│   ├── rk4_trajectory.cpp      # RK4 vs Euler
│   ├── mach_drag_demo.cpp      # Drag variation
│   ├── magnus_demo.cpp         # Magnus effect
│   ├── coriolis_demo.cpp       # Coriolis deflection
│   ├── guidance_demo.cpp       # Guidance overview
│   ├── proportional_navigation_demo.cpp  # PN variants
│   └── miss_distance_demo.cpp  # CPA analysis
├── visualizer/                 # Python plotting
│   ├── plot2d.py               # 2D matplotlib plots
│   ├── plot3d.py               # 3D Plotly visualizations
│   └── guidance_comparison.py  # Guidance comparison
├── tests/                      # Unit tests
│   ├── main_tests.cpp          # Test runner
│   ├── test_vec3.cpp           # Vec3 tests
│   ├── test_quaternion.cpp     # Quaternion tests
│   ├── test_projectile.cpp     # Projectile tests
│   └── test_guidance.cpp       # Guidance tests
├── python_demo.py              # Python demo
├── Doxyfile                    # Doxygen configuration
├── CMakeLists.txt              # Build configuration
├── README.md                   # This file
└── LICENSE                     # MIT License
```

---

## Contributing

Contributions are welcome! Areas for improvement:

- **Additional guidance laws** - Pure pursuit, lead collision, optimal control
- **Aerodynamics** - Lift models, side-force coefficients
- **Environment** - Weather modeling, non-standard atmosphere
- **Visualization** - Real-time plotting, 3D viewer improvements
- **Testing** - More test coverage, benchmarking suite

### Pull Request Process

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Write tests for new functionality
4. Ensure all tests pass (`cmake --build build --target test`)
5. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
6. Push to the branch (`git push origin feature/AmazingFeature`)
7. Open a Pull Request

### Coding Standards

- C++17 compliant code
- Doxygen comments on public APIs
- Tests for new features
- Clear commit messages

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- **JSBSim** - Flight simulation reference
- **NATO Ballistics** - Standard atmosphere models
- **Zarchan** - "Tactical and Strategic Missile Guidance"
- **Proportional Navigation** - Wikipedia and academic papers

---

## Reinforcement Learning with Gymnasium

BALLISTX provides a Gymnasium-compatible environment for training reinforcement learning agents on missile guidance tasks.

### Installation

```bash
pip install gymnasium stable-baselines3
pip install matplotlib pygame  # For rendering
```

Build BALLISTX with Python bindings:
```bash
cmake -B build -DBUILD_PYTHON_BINDINGS=ON
cmake --build build --config Debug
```

### Quick Start

```python
import sys
sys.path.insert(0, 'gymnasium')

from ballistx_env import BallistXEnv
import numpy as np

# Create environment
env = BallistXEnv()

# Reset
obs, info = env.reset()

# Run episode
for _ in range(1000):
    action = env.action_space.sample()  # Random action
    obs, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

### Environment Specification

**Observation Space** (19 dimensions):
- Missile position (x, y, z)
- Missile velocity (vx, vy, vz)
- Target position (x, y, z)
- Target velocity (vx, vy, vz)
- Relative position (dx, dy, dz)
- Relative velocity (dvx, dvy, dvz)
- Time to go (1)
- Last acceleration command (ax, ay, az)

**Action Space** (3 dimensions - Continuous):
- Acceleration command [ax, ay, az] normalized to [-1, 1]
- Scaled by `missile_max_accel` (default: 400 m/s²)

**Reward**:
- Distance reduction reward (positive for closing)
- Hit bonus (+1000 for intercept)
- Miss penalty (-500 for missing)
- Time penalty (-0.1 per step)
- Acceleration penalty (-0.01 × |accel|)

**Termination**:
- Hit: distance < proximity_threshold (default: 10m)
- Miss: distance > miss_distance_threshold (default: 500m)
- Crash: missile altitude < 0

**Truncation**:
- Max steps: 1000 (default)
- Max time: 30s (default)

### Scenarios

The `MissileGuidanceEnv` provides predefined engagement scenarios:

```python
from gymnasium.missile_guidance_env import MissileGuidanceEnv, EngagementScenario

# Head-on engagement
env = MissileGuidanceEnv(scenario=EngagementScenario.HEAD_ON)

# Tail chase
env = MissileGuidanceEnv(scenario=EngagementScenario.TAIL_CHASE)

# Crossing target
env = MissileGuidanceEnv(scenario=EngagementScenario.CROSSING)

# Maneuvering target (weave pattern)
env = MissileGuidanceEnv(scenario=EngagementScenario.MANEUVERING)

# Stationary target (ground attack)
env = MissileGuidanceEnv(scenario=EngagementScenario.STATIONARY)
```

### Training with Stable-Baselines3

```python
import sys
sys.path.insert(0, 'gymnasium')

from ballistx_env import BallistXEnv
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# Create vectorized environment
env = make_vec_env(BallistXEnv, n_envs=4)

# Train PPO agent
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100000)

# Save model
model.save("ppo_missile_guidance")

# Test trained agent
obs, info = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        obs, info = env.reset()
```

### Testing the Environment

Run the test suite:

```bash
python examples/test_ballistx_env.py
```

Expected output:
```
====================================================
BALLISTX GYMNASIUM ENVIRONMENT TEST SUITE
====================================================

Testing import...
  ✓ Import successful

Testing environment creation...
  ✓ Action space: Box(-1.0, 1.0, (3,), float32)
  ✓ Observation space: Box(-inf, inf, (19,), float32)
  ✓ Observation shape: (19,)

...

====================================================
Total: 9/9 tests passed
====================================================
```

### Random Agent Demo

```bash
python examples/random_agent_demo.py --episodes 10
```

With rendering:
```bash
python examples/random_agent_demo.py --episodes 5 --render
```

Compare random vs PN baseline:
```bash
python examples/random_agent_demo.py --compare
```

### Custom Configuration

```python
from ballistx_env import BallistXEnv, BallistXConfig

config = BallistXConfig(
    dt=0.02,
    max_steps=500,
    missile_pos=(0.0, 5000.0, 0.0),
    missile_vel=(400.0, 0.0, 0.0),
    target_pos=(12000.0, 8000.0, 0.0),
    target_vel=(250.0, 50.0, 0.0),
    missile_max_accel=500.0,
    proximity_threshold=5.0,
    miss_distance_threshold=1000.0,
    reward_hit_bonus=2000.0,
    reward_miss_penalty=-1000.0,
)

env = BallistXEnv(config=config, render_mode='human')
```

### Curriculum Learning

```python
from gymnasium.missile_guidance_env import MissileGuidanceEnv, EngagementScenario

# Start with easy scenarios
env = MissileGuidanceEnv(
    scenario=EngagementScenario.STATIONARY,
    difficulty=0.0  # Easy
)

# Train...

# Increase difficulty
env.set_difficulty(0.5)

# Train more...

# Switch to harder scenario
env.set_scenario(EngagementScenario.MANEUVERING)
env.set_difficulty(0.8)  # Hard
```

### Available Environments

| Environment | Description | Use Case |
|-------------|-------------|----------|
| `BallistXEnv` | Base environment with full configuration | Custom scenarios |
| `MissileGuidanceEnv` | Predefined scenarios + curriculum learning | Training pipelines |

### Gymnasium Files

```
ballistx_gym/
├── __init__.py
├── ballistx_env.py           # Base environment
├── missile_guidance_env.py   # Specialized guidance env
├── discrete_ballistx_env.py  # Discrete action version (for DQN)
└── renderers/
    ├── __init__.py
    └── trajectory_2d.py      # 2D matplotlib renderer

examples/
├── test_ballistx_env.py      # Environment tests
├── random_agent_demo.py      # Random agent demo
├── train_dqn.py              # DQN training script
└── train_ppo.py              # PPO training script
```

---

## RL Training

Train reinforcement learning agents to learn missile guidance.

### Installation

```bash
pip install stable-baselines3 torch
```

### DQN Training

Deep Q-Network with discrete actions (10 actions: forward, left, right, up, down, combinations).

```bash
# Quick training test (5000 timesteps)
python examples/train_dqn.py --timesteps 5000 --envs 2

# Full training (100000 timesteps)
python examples/train_dqn.py --timesteps 100000 --envs 4

# Continue training from checkpoint
python examples/train_dqn.py --load logs/dqn/models/dqn_ballistx_final.zip --timesteps 50000
```

### PPO Training

Proximal Policy Optimization with continuous actions (3D acceleration).

```bash
# Quick training test
python examples/train_ppo.py --timesteps 10000 --envs 2

# Full training
python examples/train_ppo.py --timesteps 200000 --envs 4
```

### Evaluation

```bash
# Evaluate DQN model
python examples/train_dqn.py --eval --eval-model logs/dqn/models/dqn_ballistx_final.zip --episodes 20

# Evaluate PPO model
python examples/train_ppo.py --eval --eval-model logs/ppo/models/ppo_ballistx_final.zip --episodes 20

# Evaluate with rendering
python examples/train_ppo.py --eval --eval-model logs/ppo/models/ppo_ballistx_final.zip --episodes 5 --render
```

### Model Comparison

```bash
python examples/train_ppo.py --compare --dqn-model logs/dqn/models/dqn_ballistx_final.zip --ppo-model logs/ppo/models/ppo_ballistx_final.zip --episodes 20
```

### Training Output

During training, checkpoints are saved automatically:

```
logs/dqn/
├── models/
│   ├── dqn_ballistx_10000.zip
│   ├── dqn_ballistx_20000.zip
│   ├── best/
│   └── dqn_ballistx_final.zip
├── eval/
│   ├── evaluations.npz
│   └── monitor.csv
└── monitor.csv
```

### Example Results

**Random Agent vs Trained DQN (10,000 timesteps):**

| Agent | Hit Rate | Avg Reward | Final Distance |
|-------|----------|------------|----------------|
| Random | 0% | -495 | 7993m |
| DQN (10k steps) | 0% | -3630 | 9835m |
| DQN (100k steps) | ~5% | ~+500 | ~1000m |

**Note:** Longer training (100k+ timesteps) required for significant improvement.

---

## References

### Ballistics
- [STANAG 4355](https://nato.nsolutions.nl/) - NATO Ballistics Standard
- [ISA 1976](https://www.iso.org/standard/7398.html) - International Standard Atmosphere

### Guidance
- Zarchan, P. "Tactical and Strategic Missile Guidance", AIAA
- Shneydor, N.A. "Missile Guidance and Pursuit", Springer
- [Proportional Navigation](https://en.wikipedia.org/wiki/Proportional_navigation)

### Aerodynamics
- Anderson, J.D. "Fundamentals of Aerodynamics", McGraw-Hill
- [Drag Coefficient](https://en.wikipedia.org/wiki/Drag_coefficient)

---

<div align="center">

**BALLISTX** - Advanced Ballistics & Guidance Simulation

*C++17 • Python • Real-time • Open Source*

Made with ❤️ by Emir

[GitHub](https://github.com/kodchhdayininyeri/BALLISTX) • [Issues](https://github.com/kodchhdayininyeri/BALLISTX/issues)

</div>
