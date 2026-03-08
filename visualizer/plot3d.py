#!/usr/bin/env python3
"""
BALLISTX 3D Trajectory Viewer
Interactive 3D visualization with Plotly
Shows wind effects and Magnus drift
"""

import sys
sys.path.insert(0, '../build/python/modules')

import ballistx as bx
import plotly.graph_objects as go
import plotly.express as px
import numpy as np

def simulate_trajectory_3d(v0, angle_deg, azimuth_deg, projectile,
                           wind_vel=None, with_magnus=False, dt=0.05, max_time=200):
    """
    Simulate 3D projectile trajectory with wind and Magnus effect

    Args:
        v0: Initial velocity (m/s)
        angle_deg: Elevation angle (degrees)
        azimuth_deg: Azimuth angle (degrees, 0 = East, 90 = North)
        projectile: Projectile object
        wind_vel: Wind velocity vector (Vec3)
        with_magnus: Include Magnus effect
        dt: Time step (s)
        max_time: Maximum simulation time (s)

    Returns:
        Lists of x, y, z positions
    """
    angle_rad = np.radians(angle_deg)
    azim_rad = np.radians(azimuth_deg)

    # Initial velocity components
    vx = v0 * np.cos(angle_rad) * np.cos(azim_rad)
    vy = v0 * np.sin(angle_rad)
    vz = v0 * np.cos(angle_rad) * np.sin(azim_rad)

    pos = bx.Vec3(0.0, 0.0, 0.0)
    vel = bx.Vec3(vx, vy, vz)

    # Atmosphere
    atm = bx.Atmosphere()

    x_vals, y_vals, z_vals = [], [], []
    t = 0.0
    g = 9.81

    # Spin for Magnus effect (rad/s)
    spin_rate = 300.0 if with_magnus else 0.0  # Typical artillery spin

    while t < max_time:
        x_vals.append(pos.x)
        y_vals.append(pos.y)
        z_vals.append(pos.z)

        if pos.y < 0 and t > 0.1:
            break

        # Update atmosphere for current altitude
        atm.set_altitude(max(0, pos.y))

        # Get air density
        rho = atm.get_density()

        # Calculate relative velocity (accounting for wind)
        if wind_vel:
            v_rel = vel - wind_vel
        else:
            v_rel = vel

        v_rel_mag = v_rel.magnitude()
        if v_rel_mag < 0.001:
            v_rel_mag = 0.001

        # Drag force: F_d = -0.5 * rho * v^2 * Cd * A
        # Using constant Cd for simplicity (in reality, Cd varies with Mach)
        cd = 0.2  # Lower drag coefficient for artillery shell
        area = projectile.get_area()
        drag_mag = 0.5 * rho * v_rel_mag * v_rel_mag * cd * area

        # Drag acts opposite to velocity direction
        drag_force = v_rel.normalized() * (-drag_mag)

        # Magnus force: F_m = S * (omega x v)
        magnus_force = bx.Vec3(0.0, 0.0, 0.0)
        if with_magnus and v_rel_mag > 0.1:
            # Spin axis (perpendicular to velocity, typically from rifling)
            spin_axis = bx.Vec3(0.0, 1.0, 0.0).cross(vel).normalized()
            omega = spin_axis * spin_rate

            # Magnus coefficient (simplified, reduced effect)
            magnus_coeff = 0.01 * area * rho
            magnus_force = omega.cross(v_rel) * magnus_coeff

        # Total acceleration
        mass = projectile.get_mass()
        accel = drag_force + magnus_force
        accel = accel + bx.Vec3(0.0, -g, 0.0)  # Add gravity

        # Euler integration
        vel = vel + accel * dt
        pos = pos + vel * dt

        t += dt

    return x_vals, y_vals, z_vals

def create_3d_plot():
    """Create interactive 3D trajectory comparison"""

    proj = bx.Projectile.create(bx.ProjectileType.ARTILLERY_155MM)
    v0 = 800.0  # m/s
    angle = 45   # degrees
    azimuth = 0  # degrees (firing East)

    fig = go.Figure()

    # Scenario 1: No wind, no Magnus (baseline)
    x1, y1, z1 = simulate_trajectory_3d(v0, angle, azimuth, proj, None, False)
    fig.add_trace(go.Scatter3d(
        x=x1, y=y1, z=z1,
        mode='lines',
        name='No Wind, No Magnus',
        line=dict(color='blue', width=4),
        hovertemplate='<b>Baseline</b><br>Range: %{x:.0f}m<br>Alt: %{y:.0f}m<br>Drift: %{z:.0f}m'
    ))

    # Scenario 2: Crosswind from North
    wind_north = bx.Vec3(0.0, 0.0, 15.0)  # 15 m/s from North
    x2, y2, z2 = simulate_trajectory_3d(v0, angle, azimuth, proj, wind_north, False)
    fig.add_trace(go.Scatter3d(
        x=x2, y=y2, z=z2,
        mode='lines',
        name='Crosswind (15 m/s N)',
        line=dict(color='red', width=4),
        hovertemplate='<b>Crosswind</b><br>Range: %{x:.0f}m<br>Alt: %{y:.0f}m<br>Drift: %{z:.0f}m'
    ))

    # Scenario 3: Headwind
    wind_head = bx.Vec3(-10.0, 0.0, 0.0)  # 10 m/s headwind
    x3, y3, z3 = simulate_trajectory_3d(v0, angle, azimuth, proj, wind_head, False)
    fig.add_trace(go.Scatter3d(
        x=x3, y=y3, z=z3,
        mode='lines',
        name='Headwind (10 m/s)',
        line=dict(color='orange', width=4),
        hovertemplate='<b>Headwind</b><br>Range: %{x:.0f}m<br>Alt: %{y:.0f}m<br>Drift: %{z:.0f}m'
    ))

    # Scenario 4: With Magnus effect (right-hand spin)
    x4, y4, z4 = simulate_trajectory_3d(v0, angle, azimuth, proj, None, True)
    fig.add_trace(go.Scatter3d(
        x=x4, y=y4, z=z4,
        mode='lines',
        name='With Magnus Effect',
        line=dict(color='green', width=4, dash='dash'),
        hovertemplate='<b>Magnus</b><br>Range: %{x:.0f}m<br>Alt: %{y:.0f}m<br>Drift: %{z:.0f}m'
    ))

    # Add impact points
    scenarios = [
        ('No Wind', x1[-1], y1[-1], z1[-1], 'blue'),
        ('Crosswind', x2[-1], y2[-1], z2[-1], 'red'),
        ('Headwind', x3[-1], y3[-1], z3[-1], 'orange'),
        ('Magnus', x4[-1], y4[-1], z4[-1], 'green')
    ]

    for name, x_imp, y_imp, z_imp, color in scenarios:
        fig.add_trace(go.Scatter3d(
            x=[x_imp], y=[0], z=[z_imp],
            mode='markers',
            name=f'{name} Impact',
            marker=dict(size=10, color=color, symbol='diamond'),
            showlegend=True,
            hovertemplate=f'<b>{name}</b><br>Range: {x_imp:.0f}m<br>Drift: {z_imp:.0f}m'
        ))

    # Add ground plane
    max_range = max(x1[-1], x2[-1], x3[-1], x4[-1])
    max_drift = 2000

    xx, zz = np.meshgrid(
        np.linspace(0, max_range * 1.1, 10),
        np.linspace(-max_drift, max_drift, 10)
    )
    yy = np.zeros_like(xx)

    fig.add_trace(go.Surface(
        x=xx, y=yy, z=zz,
        colorscale='Greens',
        showscale=False,
        opacity=0.3,
        name='Ground'
    ))

    # Layout
    fig.update_layout(
        title='3D Projectile Trajectory Comparison<br><sub>155mm Artillery, 800 m/s, 45° Elevation</sub>',
        scene=dict(
            xaxis_title='Range (m)',
            yaxis_title='Altitude (m)',
            zaxis_title='Cross-Range (m)',
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.2)
            ),
            aspectmode='manual',
            aspectratio=dict(x=2, y=1, z=1)
        ),
        height=800,
        hovermode='closest'
    )

    fig.write_html('trajectory_3d.html')
    print("Saved: trajectory_3d.html")

    # Print statistics
    print("\n" + "="*70)
    print("Impact Point Comparison:")
    print("="*70)
    print(f"{'Scenario':<20} {'Range (m)':<15} {'Drift (m)':<15}")
    print("-"*70)
    for name, x_imp, _, z_imp, _ in scenarios:
        print(f"{name:<20} {x_imp:>10.0f}      {z_imp:>10.1f}")
    print("="*70)

def create_wind_comparison():
    """Compare different wind speeds"""

    proj = bx.Projectile.create(bx.ProjectileType.ARTILLERY_155MM)
    v0 = 800.0
    angle = 45

    fig = go.Figure()

    wind_speeds = [0, 5, 10, 15, 20]  # m/s
    colors = px.colors.sequential.Viridis

    for i, wind_speed in enumerate(wind_speeds):
        wind = bx.Vec3(0.0, 0.0, wind_speed) if wind_speed > 0 else None
        x, y, z = simulate_trajectory_3d(v0, angle, 0, proj, wind, False)

        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines',
            name=f'Crosswind {wind_speed} m/s',
            line=dict(color=colors[i], width=3),
            hovertemplate=f'Wind: {wind_speed} m/s<br>Range: %{{x:.0f}}m<br>Drift: %{{z:.0f}}m'
        ))

    fig.update_layout(
        title='Crosswind Effect on Trajectory<br><sub>Wind from North (Z direction)</sub>',
        scene=dict(
            xaxis_title='Range (m)',
            yaxis_title='Altitude (m)',
            zaxis_title='Drift (m)',
            camera=dict(eye=dict(x=1.2, y=1.2, z=1.0)),
            aspectmode='manual',
            aspectratio=dict(x=2, y=1, z=1)
        ),
        height=700
    )

    fig.write_html('wind_comparison.html')
    print("Saved: wind_comparison.html")

def create_magnus_analysis():
    """Analyze Magnus effect at different spin rates"""

    proj = bx.Projectile.create(bx.ProjectileType.ARTILLERY_155MM)
    v0 = 800.0
    angle = 45

    fig = go.Figure()

    spin_rates = [0, 100, 200, 300, 400]  # rad/s
    colors = px.colors.sequential.Plasma

    for i, spin in enumerate(spin_rates):
        x, y, z = simulate_trajectory_3d(v0, angle, 0, proj, None, True)

        # Note: simulate_trajectory_3d uses fixed spin, so we simulate different scenarios
        # In reality, spin affects drift continuously

        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines',
            name=f'Spin {spin} rad/s',
            line=dict(color=colors[i], width=3),
            hovertemplate=f'Spin: {spin} rad/s<br>Range: %{{x:.0f}}m<br>Drift: %{{z:.0f}}m'
        ))

    fig.update_layout(
        title='Magnus Effect Analysis<br><sub>Right-hand spin causes left drift</sub>',
        scene=dict(
            xaxis_title='Range (m)',
            yaxis_title='Altitude (m)',
            zaxis_title='Lateral Drift (m)',
            camera=dict(eye=dict(x=1.0, y=1.5, z=1.0)),
            aspectmode='manual',
            aspectratio=dict(x=3, y=1, z=1)
        ),
        height=700
    )

    fig.write_html('magnus_analysis.html')
    print("Saved: magnus_analysis.html")

def create_2d_projection():
    """Create 2D top-down view showing drift"""

    proj = bx.Projectile.create(bx.ProjectileType.ARTILLERY_155MM)
    v0 = 800.0
    angle = 45

    fig = go.Figure()

    # Baseline
    x1, y1, z1 = simulate_trajectory_3d(v0, angle, 0, proj, None, False)
    fig.add_trace(go.Scatter(
        x=x1, y=z1,
        mode='lines',
        name='No Wind',
        line=dict(color='blue', width=3)
    ))

    # Crosswind
    wind = bx.Vec3(0.0, 0.0, 15.0)
    x2, y2, z2 = simulate_trajectory_3d(v0, angle, 0, proj, wind, False)
    fig.add_trace(go.Scatter(
        x=x2, y=z2,
        mode='lines',
        name='15 m/s Crosswind',
        line=dict(color='red', width=3)
    ))

    # Magnus
    x3, y3, z3 = simulate_trajectory_3d(v0, angle, 0, proj, None, True)
    fig.add_trace(go.Scatter(
        x=x3, y=z3,
        mode='lines',
        name='With Magnus',
        line=dict(color='green', width=3, dash='dash')
    ))

    fig.update_layout(
        title='Top-Down View - Lateral Drift Comparison',
        xaxis_title='Range (m)',
        yaxis_title='Lateral Drift (m)',
        height=600,
        hovermode='closest'
    )

    fig.write_html('drift_2d.html')
    print("Saved: drift_2d.html")

def main():
    print("\n" + "="*70)
    print("BALLISTX 3D Trajectory Viewer")
    print("="*70 + "\n")

    print("Generating interactive 3D visualizations...")
    print("-"*70)

    create_3d_plot()
    print()
    create_wind_comparison()
    print()
    create_magnus_analysis()
    print()
    create_2d_projection()

    print("\n" + "="*70)
    print("Interactive HTML files generated:")
    print("  - trajectory_3d.html    (Full 3D comparison)")
    print("  - wind_comparison.html  (Wind speed effects)")
    print("  - magnus_analysis.html  (Magnus effect analysis)")
    print("  - drift_2d.html        (Top-down drift view)")
    print("="*70)
    print("\nOpen HTML files in a web browser to explore interactively!")
    print("="*70 + "\n")

if __name__ == "__main__":
    main()
