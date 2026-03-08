#!/usr/bin/env python3
"""
BALLISTX 2D Trajectory Plotter
Visualizes projectile trajectories with different launch angles
"""

import sys
sys.path.insert(0, '../build/python/modules')

import ballistx as bx
import matplotlib.pyplot as plt
import numpy as np

def simulate_trajectory(v0, angle_deg, projectile, dt=0.01, max_time=100):
    """
    Simulate projectile trajectory

    Args:
        v0: Initial velocity (m/s)
        angle_deg: Launch angle (degrees)
        projectile: Projectile object
        dt: Time step (s)
        max_time: Maximum simulation time (s)

    Returns:
        List of (x, y) positions
    """
    # Convert angle to radians
    angle_rad = np.radians(angle_deg)

    # Initial conditions
    pos = bx.Vec3(0.0, 0.0, 0.0)
    vel = bx.Vec3(
        v0 * np.cos(angle_rad),
        v0 * np.sin(angle_rad),
        0.0
    )

    trajectory = []
    t = 0.0
    g = 9.81

    while t < max_time:
        # Store position
        trajectory.append((pos.x, pos.y))

        # Check if hit ground
        if pos.y < 0 and t > 0.1:
            break

        # Simple Euler integration with gravity
        accel = bx.Vec3(0.0, -g, 0.0)
        vel = vel + accel * dt
        pos = pos + vel * dt

        t += dt

    return trajectory

def plot_trajectories():
    """Plot trajectories for different launch angles"""

    # Projectile: 155mm artillery shell
    proj = bx.Projectile.create(bx.ProjectileType.ARTILLERY_155MM)

    # Initial velocity
    v0 = 800.0  # m/s

    # Launch angles to compare
    angles = [15, 30, 45, 60, 75]

    # Colors for each angle
    colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7']

    fig, ax = plt.subplots(figsize=(12, 8))

    print("Simulating trajectories...")
    print("-" * 50)

    max_range = 0
    max_height = 0

    for angle, color in zip(angles, colors):
        traj = simulate_trajectory(v0, angle, proj)

        x_vals = [p[0] for p in traj]
        y_vals = [p[1] for p in traj]

        range_dist = x_vals[-1]
        height = max(y_vals)

        max_range = max(max_range, range_dist)
        max_height = max(max_height, height)

        print(f"Angle: {angle:2d}deg | Range: {range_dist/1000:6.2f} km | Max Height: {height:6.1f} m")

        # Plot trajectory
        ax.plot(x_vals, y_vals,
                linewidth=2.5,
                color=color,
                label=f'{angle}° (R={range_dist/1000:.1f}km, H={height:.0f}m)',
                alpha=0.8)

        # Mark impact point
        ax.scatter(x_vals[-1], 0,
                   color=color,
                   s=100,
                   marker='v',
                   edgecolors='black',
                   linewidths=1,
                   zorder=5)

    print("-" * 50)
    print(f"Maximum Range: {max_range/1000:.2f} km")
    print(f"Maximum Height: {max_height:.1f} m")

    # Styling
    ax.set_xlabel('Range (m)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Altitude (m)', fontsize=14, fontweight='bold')
    ax.set_title(f'Projectile Trajectories - {v0} m/s Initial Velocity\n155mm Artillery Shell',
                 fontsize=16, fontweight='bold')
    ax.legend(fontsize=11, loc='upper right', framealpha=0.95)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_xlim(0, max_range * 1.05)
    ax.set_ylim(0, max_height * 1.1)

    # Add ground line
    ax.axhline(y=0, color='brown', linewidth=3, alpha=0.5)
    ax.text(max_range * 0.02, -max_height * 0.05, 'GROUND',
            color='brown', fontweight='bold', fontsize=10)

    # Add max range marker
    ax.axvline(x=max_range, color='red', linestyle=':', alpha=0.5)
    ax.text(max_range, max_height * 0.5, f' Max Range\n{max_range/1000:.1f} km',
            color='red', fontsize=10, ha='right')

    plt.tight_layout()
    plt.savefig('trajectory_2d.png', dpi=150, bbox_inches='tight')
    print("\nSaved: trajectory_2d.png")
    plt.close()

def plot_velocity_profile():
    """Plot velocity vs range for different angles"""

    proj = bx.Projectile.create(bx.ProjectileType.ARTILLERY_155MM)
    v0 = 800.0
    angles = [30, 45, 60]

    fig, ax = plt.subplots(figsize=(12, 8))

    for angle in angles:
        angle_rad = np.radians(angle)
        pos = bx.Vec3(0.0, 0.0, 0.0)
        vel = bx.Vec3(v0 * np.cos(angle_rad), v0 * np.sin(angle_rad), 0.0)

        ranges = []
        velocities = []

        t = 0.0
        dt = 0.01
        while t < 100:
            if pos.y < 0 and t > 0.1:
                break

            ranges.append(pos.x)
            velocities.append(vel.magnitude())

            accel = bx.Vec3(0.0, -9.81, 0.0)
            vel = vel + accel * dt
            pos = pos + vel * dt
            t += dt

        ax.plot(ranges, velocities,
                linewidth=2.5,
                label=f'{angle}° launch',
                alpha=0.8)

    ax.set_xlabel('Range (m)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Velocity (m/s)', fontsize=14, fontweight='bold')
    ax.set_title('Velocity Profile vs Range', fontsize=16, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3, linestyle='--')

    plt.tight_layout()
    plt.savefig('velocity_profile.png', dpi=150, bbox_inches='tight')
    print("Saved: velocity_profile.png")
    plt.close()

def plot_energy_profile():
    """Plot kinetic energy vs altitude"""

    proj = bx.Projectile.create(bx.ProjectileType.ARTILLERY_155MM)
    v0 = 800.0
    angle = 45

    angle_rad = np.radians(angle)
    pos = bx.Vec3(0.0, 0.0, 0.0)
    vel = bx.Vec3(v0 * np.cos(angle_rad), v0 * np.sin(angle_rad), 0.0)

    altitudes = []
    energies = []

    t = 0.0
    dt = 0.01
    mass = proj.get_mass()

    while t < 100:
        if pos.y < 0 and t > 0.1:
            break

        ke = 0.5 * mass * vel.magnitude() ** 2
        pe = mass * 9.81 * pos.y
        total_e = ke + pe

        altitudes.append(pos.y)
        energies.append({
            'kinetic': ke / 1e6,  # Convert to MJ
            'potential': pe / 1e6,
            'total': total_e / 1e6
        })

        accel = bx.Vec3(0.0, -9.81, 0.0)
        vel = vel + accel * dt
        pos = pos + vel * dt
        t += dt

    fig, ax = plt.subplots(figsize=(12, 8))

    ke_vals = [e['kinetic'] for e in energies]
    pe_vals = [e['potential'] for e in energies]
    total_vals = [e['total'] for e in energies]

    ax.plot(altitudes, ke_vals, linewidth=2.5, label='Kinetic Energy', color='blue', alpha=0.8)
    ax.plot(altitudes, pe_vals, linewidth=2.5, label='Potential Energy', color='green', alpha=0.8)
    ax.plot(altitudes, total_vals, linewidth=2.5, label='Total Energy', color='red', linestyle='--', alpha=0.8)

    ax.set_xlabel('Altitude (m)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Energy (MJ)', fontsize=14, fontweight='bold')
    ax.set_title(f'Energy Profile vs Altitude - {angle}° Launch Angle', fontsize=16, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3, linestyle='--')

    plt.tight_layout()
    plt.savefig('energy_profile.png', dpi=150, bbox_inches='tight')
    print("Saved: energy_profile.png")
    plt.close()

def main():
    print("\n" + "="*70)
    print("BALLISTX 2D Trajectory Plotter")
    print("="*70 + "\n")

    # Generate plots
    plot_trajectories()
    print()
    plot_velocity_profile()
    print()
    plot_energy_profile()

    print("\n" + "="*70)
    print("Visualization Complete!")
    print("Generated files:")
    print("  - trajectory_2d.png")
    print("  - velocity_profile.png")
    print("  - energy_profile.png")
    print("="*70 + "\n")

if __name__ == "__main__":
    main()
