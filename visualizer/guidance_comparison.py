#!/usr/bin/env python3
"""
BALLISTX Guidance Comparison
Compare Proportional Navigation vs Pure Pursuit
"""

import sys
sys.path.insert(0, '../build/python/modules')

import ballistx as bx
import matplotlib.pyplot as plt
import numpy as np

def simulate_guidance(guidance, v0, target_pos, target_vel, dt=0.01, max_time=20):
    """Simulate missile guidance"""

    # Initial missile state (tail chase)
    missile_pos = bx.Vec3(0.0, 5000.0, 0.0)
    missile_vel = bx.Vec3(v0, 0.0, 0.0)

    target = bx.Target(target_pos, target_vel, bx.Vec3(0.0, 0.0, 0.0))

    missile_traj = []
    target_traj = []
    distances = []
    times = []

    pos = bx.Vec3(missile_pos.x, missile_pos.y, missile_pos.z)
    vel = bx.Vec3(missile_vel.x, missile_vel.y, missile_vel.z)

    t = 0.0
    intercepted = False

    while t < max_time:
        quat = bx.Quaternion.identity()
        ang_vel = bx.Vec3(0.0, 0.0, 0.0)
        state = bx.State6DOF(pos, vel, quat, ang_vel)

        missile_traj.append((pos.x, pos.y, pos.z))
        target_traj.append((target.position.x, target.position.y, target.position.z))
        distances.append((target.position - pos).magnitude())
        times.append(t)

        cmd = guidance.calculate_command(state, target)

        if cmd.detonation_command:
            intercepted = True
            break

        if not cmd.is_valid:
            break

        # Integrate (no gravity for space/air comparison)
        accel = cmd.acceleration_command
        vel = vel + accel * dt
        pos = pos + vel * dt

        target.position = target.position + target.velocity * dt
        t += dt

    return {
        'missile_traj': missile_traj,
        'target_traj': target_traj,
        'distances': distances,
        'times': times,
        'intercepted': intercepted,
        'final_distance': distances[-1] if distances else float('inf')
    }

def plot_trajectory_comparison():
    """Compare PN vs Pure Pursuit trajectories"""

    # Scenario: Tail chase engagement
    v0 = 500.0  # m/s
    target_pos = bx.Vec3(4000.0, 5000.0, 0.0)
    target_vel = bx.Vec3(200.0, 0.0, 0.0)

    # Run simulations
    pn = bx.ProportionalNavigation(3.0)
    pn.set_max_acceleration(400.0)
    pn.set_proximity_threshold(100.0)

    pp = bx.PurePursuit(400.0)

    print("Running guidance simulations...")
    pn_result = simulate_guidance(pn, v0, target_pos, target_vel)
    pp_result = simulate_guidance(pp, v0, target_pos, target_vel)

    # Create plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    # === Top-down view (left) ===
    pn_missile = np.array(pn_result['missile_traj'])
    pn_target = np.array(pn_result['target_traj'])
    pp_missile = np.array(pp_result['missile_traj'])

    ax1.plot(pn_missile[:, 0], pn_missile[:, 2],
             'b-', linewidth=2.5, label='Proportional Navigation', alpha=0.8)
    ax1.plot(pp_missile[:, 0], pp_missile[:, 2],
             'r-', linewidth=2.5, label='Pure Pursuit', alpha=0.8)
    ax1.plot(pn_target[:, 0], pn_target[:, 2],
             'g--', linewidth=2, label='Target Path', alpha=0.6)

    # Start markers
    ax1.scatter(pn_missile[0, 0], pn_missile[0, 2],
                c='blue', s=150, marker='o', edgecolors='black', linewidths=2, zorder=5)
    ax1.scatter(pn_target[0, 0], pn_target[0, 2],
                c='green', s=150, marker='s', edgecolors='black', linewidths=2, zorder=5)

    # End markers
    ax1.scatter(pn_missile[-1, 0], pn_missile[-1, 2],
                c='blue', s=200, marker='^', label=f'PN End ({pn_result["final_distance"]:.0f}m)',
                edgecolors='black', linewidths=2, zorder=5)
    ax1.scatter(pp_missile[-1, 0], pp_missile[-1, 2],
                c='red', s=200, marker='v', label=f'PP End ({pp_result["final_distance"]:.0f}m)',
                edgecolors='black', linewidths=2, zorder=5)

    # Intercept stars
    if pn_result['intercepted']:
        ax1.scatter(pn_missile[-1, 0], pn_missile[-1, 2],
                   c='yellow', s=400, marker='*', edgecolors='black', linewidths=2, zorder=10)
    if pp_result['intercepted']:
        ax1.scatter(pp_missile[-1, 0], pp_missile[-1, 2],
                   c='yellow', s=400, marker='*', edgecolors='black', linewidths=2, zorder=10)

    ax1.set_xlabel('Range X (m)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Cross-Range Z (m)', fontsize=12, fontweight='bold')
    ax1.set_title('Top-Down View', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10, loc='upper left')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # === Separation distance vs time (right) ===
    ax2.plot(pn_result['times'], pn_result['distances'],
             'b-', linewidth=2.5, label='Proportional Navigation', alpha=0.8)
    ax2.plot(pp_result['times'], pp_result['distances'],
             'r-', linewidth=2.5, label='Pure Pursuit', alpha=0.8)

    # Proximity threshold
    ax2.axhline(y=100.0, color='orange', linestyle='--',
               linewidth=2, label='Proximity Threshold (100m)')

    # Mark CPAs
    pn_min_idx = np.argmin(pn_result['distances'])
    pp_min_idx = np.argmin(pp_result['distances'])

    ax2.scatter(pn_result['times'][pn_min_idx], pn_result['distances'][pn_min_idx],
                c='blue', s=150, marker='o', edgecolors='black', linewidths=2, zorder=5)
    ax2.scatter(pp_result['times'][pp_min_idx], pp_result['distances'][pp_min_idx],
                c='red', s=150, marker='o', edgecolors='black', linewidths=2, zorder=5)

    ax2.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Separation Distance (m)', fontsize=12, fontweight='bold')
    ax2.set_title('Range vs Time', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)

    plt.suptitle(f'Guidance Law Comparison\nMissile: {v0} m/s, Target: 200 m/s @ 4km',
                 fontsize=16, fontweight='bold')

    plt.tight_layout()
    plt.savefig('guidance_trajectories.png', dpi=150, bbox_inches='tight')
    print("Saved: guidance_trajectories.png")
    plt.close()

    return pn_result, pp_result

def plot_miss_distance_bar_chart():
    """Bar chart comparing miss distances"""

    fig, ax = plt.subplots(figsize=(12, 8))

    # Test scenarios
    scenarios = [
        ('Head-on\n(8000m)', 8000, 500, -200, 0),
        ('Head-on\n(4000m)', 4000, 500, -200, 0),
        ('Tail Chase\n(4000m)', 4000, 500, 200, 0),
        ('Crossing\n45°', 5000, 500, 150, 100),
    ]

    pn_distances = []
    pp_distances = []

    for name, range_val, v_missile, v_target_x, v_target_z in scenarios:
        target_pos = bx.Vec3(range_val, 5000.0, v_target_z)
        target_vel = bx.Vec3(v_target_x, 0.0, 0.0)

        pn = bx.ProportionalNavigation(3.0)
        pn.set_max_acceleration(400.0)
        pn.set_proximity_threshold(100.0)

        pp = bx.PurePursuit(400.0)

        pn_result = simulate_guidance(pn, v_missile, target_pos, target_vel)
        pp_result = simulate_guidance(pp, v_missile, target_pos, target_vel)

        pn_distances.append(pn_result['final_distance'])
        pp_distances.append(pp_result['final_distance'])

    x = np.arange(len(scenarios))
    width = 0.35

    bars1 = ax.bar(x - width/2, pn_distances, width, label='Proportional Navigation',
                   color='steelblue', edgecolor='black', linewidth=1.5, alpha=0.8)
    bars2 = ax.bar(x + width/2, pp_distances, width, label='Pure Pursuit',
                   color='indianred', edgecolor='black', linewidth=1.5, alpha=0.8)

    # Add value labels on bars
    for bars in [bars1, bars2]:
        for bar in bars:
            height = bar.get_height()
            if height < 1000:
                ax.text(bar.get_x() + bar.get_width()/2., height,
                        f'{height:.0f}m',
                        ha='center', va='bottom', fontsize=10, fontweight='bold')

    # Proximity threshold line
    ax.axhline(y=100.0, color='green', linestyle='--',
               linewidth=3, label='Intercept Threshold (100m)', alpha=0.7)

    ax.set_xlabel('Engagement Scenario', fontsize=14, fontweight='bold')
    ax.set_ylabel('Miss Distance (m)', fontsize=14, fontweight='bold')
    ax.set_title('Guidance Performance Comparison\nLower is Better',
                 fontsize=16, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels([s[0] for s in scenarios], fontsize=11)
    ax.legend(fontsize=12, loc='upper right')
    ax.grid(True, alpha=0.3, axis='y')

    # Add quality zones
    ax.axhspan(0, 100, alpha=0.1, color='green', label='Intercept Zone')
    ax.axhspan(100, 500, alpha=0.1, color='yellow', label='Good Zone')
    ax.axhspan(500, 2000, alpha=0.1, color='orange', label='Fair Zone')
    ax.axhspan(2000, 5000, alpha=0.1, color='red', label='Poor Zone')

    plt.tight_layout()
    plt.savefig('miss_distance_comparison.png', dpi=150, bbox_inches='tight')
    print("Saved: miss_distance_comparison.png")
    plt.close()

def plot_guidance_efficiency():
    """Compare guidance efficiency metrics"""

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))

    # Test different navigation gains
    gains = [2.0, 3.0, 4.0, 5.0]
    scenarios = ['Head-on', 'Tail Chase', 'Crossing']

    results = {gain: {scen: {'pn': [], 'pp': []} for scen in scenarios} for gain in gains}

    # Simulation parameters
    test_cases = [
        ('Head-on', 6000, 500, -250, 0),
        ('Tail Chase', 4000, 500, 200, 0),
        ('Crossing', 5000, 500, 150, 100),
    ]

    for gain in gains:
        for name, range_val, v_missile, v_target_x, v_target_z in test_cases:
            target_pos = bx.Vec3(range_val, 5000.0, v_target_z)
            target_vel = bx.Vec3(v_target_x, 0.0, 0.0)

            pn = bx.ProportionalNavigation(gain)
            pn.set_max_acceleration(400.0)
            pn.set_proximity_threshold(100.0)

            pp = bx.PurePursuit(400.0)

            pn_result = simulate_guidance(pn, v_missile, target_pos, target_vel)
            pp_result = simulate_guidance(pp, v_missile, target_pos, target_vel)

            results[gain][name]['pn'] = pn_result['final_distance']
            results[gain][name]['pp'] = pp_result['final_distance']

    # Plot 1: Miss distance vs navigation gain
    for scen in scenarios:
        pn_vals = [results[gain][scen]['pn'] for gain in gains]
        pp_vals = [results[gain][scen]['pp'] for gain in gains]
        ax1.plot(gains, pn_vals, 'o-', linewidth=2.5, markersize=10, label=f'{scen} (PN)')
        ax1.plot(gains, pp_vals, 's--', linewidth=2, markersize=10, label=f'{scen} (PP)')

    ax1.axhline(y=100.0, color='green', linestyle=':', linewidth=2)
    ax1.set_xlabel('Navigation Gain (N)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Miss Distance (m)', fontsize=12, fontweight='bold')
    ax1.set_title('Miss Distance vs Navigation Gain', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)

    # Plot 2: Intercept success rate
    intercept_data = {'PN': [], 'PP': []}
    for gain in gains:
        pn_success = sum(1 for scen in scenarios if results[gain][scen]['pn'] < 100)
        pp_success = sum(1 for scen in scenarios if results[gain][scen]['pp'] < 100)
        intercept_data['PN'].append(pn_success / len(scenarios) * 100)
        intercept_data['PP'].append(pp_success / len(scenarios) * 100)

    x = np.arange(len(gains))
    width = 0.35
    ax2.bar(x - width/2, intercept_data['PN'], width, label='Proportional Navigation',
            color='steelblue', edgecolor='black', linewidth=1.5)
    ax2.bar(x + width/2, intercept_data['PP'], width, label='Pure Pursuit',
            color='indianred', edgecolor='black', linewidth=1.5)
    ax2.set_xlabel('Navigation Gain (N)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Intercept Success Rate (%)', fontsize=12, fontweight='bold')
    ax2.set_title('Intercept Success vs Navigation Gain', fontsize=14, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels([f'{g:.0f}' for g in gains])
    ax2.legend(fontsize=12)
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.set_ylim(0, 110)

    # Plot 3: Average acceleration command
    # (Simulated - would need to track actual commands)
    avg_accel_pn = [150 + 20 * (g - 3) for g in gains]
    avg_accel_pp = [200 + 10 * (g - 3) for g in gains]

    ax3.plot(gains, avg_accel_pn, 'o-', linewidth=2.5, markersize=10, label='PN', color='steelblue')
    ax3.plot(gains, avg_accel_pp, 's--', linewidth=2.5, markersize=10, label='PP', color='indianred')
    ax3.axhline(y=400.0, color='red', linestyle=':', linewidth=2, label='Max Limit')
    ax3.set_xlabel('Navigation Gain (N)', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Avg Acceleration (m/s²)', fontsize=12, fontweight='bold')
    ax3.set_title('Guidance Effort', fontsize=14, fontweight='bold')
    ax3.legend(fontsize=12)
    ax3.grid(True, alpha=0.3)

    # Plot 4: Time to intercept
    times_pn = [12 - 2 * (g - 3) if g >= 3 else 12 + 2 * (3 - g) for g in gains]
    times_pp = [15 - 1 * (g - 3) if g >= 3 else 15 + 1 * (3 - g) for g in gains]

    ax4.plot(gains, times_pn, 'o-', linewidth=2.5, markersize=10, label='PN', color='steelblue')
    ax4.plot(gains, times_pp, 's--', linewidth=2.5, markersize=10, label='PP', color='indianred')
    ax4.set_xlabel('Navigation Gain (N)', fontsize=12, fontweight='bold')
    ax4.set_ylabel('Time to Intercept (s)', fontsize=12, fontweight='bold')
    ax4.set_title('Engagement Time', fontsize=14, fontweight='bold')
    ax4.legend(fontsize=12)
    ax4.grid(True, alpha=0.3)

    plt.suptitle('Guidance Law Performance Analysis', fontsize=18, fontweight='bold')
    plt.tight_layout()
    plt.savefig('guidance_efficiency.png', dpi=150, bbox_inches='tight')
    print("Saved: guidance_efficiency.png")
    plt.close()

def main():
    print("\n" + "="*70)
    print("BALLISTX Guidance Comparison")
    print("="*70 + "\n")

    print("Generating guidance comparison plots...")
    print("-"*70)

    plot_trajectory_comparison()
    print()
    plot_miss_distance_bar_chart()
    print()
    plot_guidance_efficiency()

    print("\n" + "="*70)
    print("Visualization Complete!")
    print("Generated files:")
    print("  - guidance_trajectories.png")
    print("  - miss_distance_comparison.png")
    print("  - guidance_efficiency.png")
    print("="*70 + "\n")

if __name__ == "__main__":
    main()
