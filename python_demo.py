#!/usr/bin/env python3
"""
BALLISTX Python Demo
Demonstration of Python bindings for ballistic simulation library
"""

import sys
sys.path.insert(0, 'build/python/Debug')

import ballistx_py as bx

def main():
    print("\n" + "="*70)
    print("BALLISTX Python Bindings Demo")
    print("="*70 + "\n")

    # ========================================================================
    # Vec3 Operations
    # ========================================================================
    print("1. Vec3 Operations:")
    print("-" * 40)

    v1 = bx.Vec3(3.0, 4.0, 0.0)
    v2 = bx.Vec3(1.0, 2.0, 3.0)

    print(f"   v1 = {v1}")
    print(f"   v2 = {v2}")
    print(f"   Magnitude v1: {v1.magnitude()}")
    print(f"   v1 + v2 = {v1 + v2}")
    print(f"   v1 * 2.0 = {v1 * 2.0}")
    print(f"   Dot product: {v1.dot(v2)}")
    print(f"   Cross product: {v1.cross(v2)}")
    print(f"   Normalized v1: {v1.normalized()}")

    # ========================================================================
    # Quaternion Operations
    # ========================================================================
    print("\n2. Quaternion Operations:")
    print("-" * 40)

    q = bx.Quaternion.from_euler(0.0, 0.0, 1.57)  # 90 degrees yaw
    print(f"   Quaternion from Euler (0, 0, 90°): {q.magnitude()}")
    print(f"   Identity quaternion: {bx.Quaternion.identity().magnitude()}")

    # ========================================================================
    # 6-DOF State
    # ========================================================================
    print("\n3. 6-DOF State:")
    print("-" * 40)

    state = bx.State6DOF(
        bx.Vec3(0.0, 5000.0, 0.0),      # Position (m)
        bx.Vec3(400.0, 0.0, 0.0),       # Velocity (m/s)
        bx.Quaternion.identity(),       # Orientation
        bx.Vec3(0.0, 0.0, 0.0)          # Angular velocity (rad/s)
    )

    print(f"   Position: {state.get_position()}")
    print(f"   Velocity: {state.get_velocity()}")
    print(f"   Speed: {state.get_velocity().magnitude()} m/s")

    # ========================================================================
    # Target
    # ========================================================================
    print("\n4. Target:")
    print("-" * 40)

    target = bx.Target(
        bx.Vec3(8000.0, 5200.0, 200.0),  # Position (m)
        bx.Vec3(200.0, 0.0, 0.0),        # Velocity (m/s)
        bx.Vec3(0.0, 0.0, 0.0)           # Acceleration (m/s²)
    )

    print(f"   Position: {target.position}")
    print(f"   Velocity: {target.velocity}")
    print(f"   Speed: {target.velocity.magnitude()} m/s")
    print(f"   Is moving: {target.is_moving()}")

    # Predict target position in 5 seconds
    pred_pos = target.predict_position(5.0)
    print(f"   Predicted position (5s): {pred_pos}")

    # ========================================================================
    # Proportional Navigation Guidance
    # ========================================================================
    print("\n5. Proportional Navigation:")
    print("-" * 40)

    pn = bx.ProportionalNavigation(3.0)  # N = 3
    print(f"   Guidance type: {pn.get_name()}")

    cmd = pn.calculate_command(state, target)
    print(f"   Command valid: {cmd.is_valid}")
    print(f"   Acceleration command: {cmd.acceleration_command.magnitude()} m/s²")
    print(f"   Time to go: {cmd.time_to_go:.2f} s")

    # ========================================================================
    # Miss Distance / CPA Calculation
    # ========================================================================
    print("\n6. Miss Distance / CPA:")
    print("-" * 40)

    cpa_result = bx.MissDistance.calculate_cpa(state, target)

    print(f"   Miss distance: {cpa_result.miss_distance:.2f} m")
    print(f"   Time to CPA: {cpa_result.time_to_cpa:.2f} s")
    print(f"   Intercepted: {cpa_result.intercepted}")
    print(f"   Closing velocity: {cpa_result.closing_velocity:.2f} m/s")

    quality = bx.MissDistance.evaluate_quality(cpa_result.miss_distance)
    description = bx.MissDistance.get_quality_description(cpa_result.miss_distance)
    print(f"   Quality: {quality}/100 - {description}")

    # ========================================================================
    # Real-time CPA Tracking
    # ========================================================================
    print("\n7. Real-time CPA Tracking:")
    print("-" * 40)

    tracker = bx.MissDistanceTracker()

    # Simulate some updates
    for i in range(5):
        missile_pos = bx.Vec3(float(i * 800), 5000.0, 0.0)
        tgt_pos = bx.Vec3(8000.0 + float(i * 500), 5200.0, 200.0)
        tracker.update(missile_pos, tgt_pos, float(i))

    result = tracker.get_result()
    print(f"   Minimum distance: {result.miss_distance:.2f} m")
    print(f"   Time at minimum: {result.time_to_cpa:.2f} s")

    # ========================================================================
    # Atmosphere
    # ========================================================================
    print("\n8. Atmosphere (ISA Model):")
    print("-" * 40)

    atm = bx.Atmosphere(5000.0)  # 5000 m altitude
    print(f"   Altitude: {atm.get_altitude()} m")
    print(f"   Temperature: {atm.get_temperature():.2f} K")
    print(f"   Pressure: {atm.get_pressure():.2f} Pa")
    print(f"   Density: {atm.get_density():.4f} kg/m³")
    print(f"   Speed of sound: {atm.get_speed_of_sound():.2f} m/s")

    # ========================================================================
    # Projectile Types
    # ========================================================================
    print("\n9. Projectile Types:")
    print("-" * 40)

    proj = bx.Projectile.create(bx.ProjectileType.MISSILE_AIR_TO_AIR)
    print(f"   Type: {proj.get_name()}")
    print(f"   Mass: {proj.get_mass():.2f} kg")
    print(f"   Diameter: {proj.get_diameter()*1000:.1f} mm")
    print(f"   Drag coefficient: {proj.get_drag_coefficient():.3f}")
    print(f"   Area: {proj.get_area():.4f} m²")
    print(f"   Valid: {proj.is_valid()}")

    print("\n" + "="*70)
    print("Demo Complete!")
    print("="*70 + "\n")

if __name__ == "__main__":
    main()
