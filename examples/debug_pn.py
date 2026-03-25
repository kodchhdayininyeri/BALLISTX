"""
Debug PN Guidance - Print detailed telemetry to understand what's happening
"""

import sys
from pathlib import Path

# Add ballistx_gym package to path
gym_path = Path(__file__).parent.parent
sys.path.insert(0, str(gym_path))

import numpy as np
from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

def debug_pn():
    """Debug PN guidance step by step."""

    config = BallistXConfig(
        dt=0.02,
        max_steps=500,
        max_time=15.0,
        missile_pos=(0.0, 5000.0, 0.0),
        missile_vel=(350.0, 0.0, 0.0),
        missile_max_accel=400.0,
        target_pos=(8000.0, 5000.0, 0.0),
        target_vel=(200.0, 0.0, 0.0),
        proximity_threshold=15.0,
        miss_distance_threshold=20000.0,
        reward_miss_penalty=-100.0,
    )

    env = BallistXEnv(config=config)
    obs, info = env.reset()

    print("=" * 80)
    print("PN GUIDANCE DEBUG TRACE")
    print("=" * 80)
    print(f"\nInitial State:")
    print(f"  Missile Pos: ({info['missile_position'][0]:.1f}, {info['missile_position'][1]:.1f}, {info['missile_position'][2]:.1f})")
    print(f"  Target Pos:  ({info['target_position'][0]:.1f}, {info['target_position'][1]:.1f}, {info['target_position'][2]:.1f})")
    print(f"  Distance: {info['distance']:.1f}m")
    print(f"  Closing Speed: {info['closing_speed']:.1f} m/s")

    print(f"\nBaseline Acceleration Command (first step):")
    baseline = info['baseline_accel']
    print(f"  ax = {baseline[0]:.2f} m/s^2")
    print(f"  ay = {baseline[1]:.2f} m/s^2")
    print(f"  az = {baseline[2]:.2f} m/s^2")
    print(f"  magnitude = {np.linalg.norm(baseline):.2f} m/s^2")

    print("\n" + "-" * 80)
    print("Running PN guidance for 10 steps...")
    print("-" * 80)

    for step in range(min(10, config.max_steps)):
        # Use PN baseline
        baseline = info['baseline_accel']
        action = baseline / config.missile_max_accel

        print(f"\nStep {step}:")
        print(f"  Action: [{action[0]:.3f}, {action[1]:.3f}, {action[2]:.3f}]")
        print(f"  Distance: {info['distance']:.1f}m")
        print(f"  Baseline accel: [{baseline[0]:.2f}, {baseline[1]:.2f}, {baseline[2]:.2f}]")

        obs, reward, term, trunc, info = env.step(action)

        if term or trunc:
            print(f"\nTerminated at step {step}: {info['terminated_reason']}")
            print(f"Final distance: {info['distance']:.1f}m")
            break

    if not (term or trunc):
        print(f"\nCompleted 10 steps without termination")
        print(f"Final distance: {info['distance']:.1f}m")

    env.close()

if __name__ == '__main__':
    debug_pn()
