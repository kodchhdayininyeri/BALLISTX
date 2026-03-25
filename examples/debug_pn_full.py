"""
Debug PN Guidance - Full episode with timing analysis
"""

import sys
from pathlib import Path

# Add ballistx_gym package to path
gym_path = Path(__file__).parent.parent
sys.path.insert(0, str(gym_path))

import numpy as np
from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

def analyze_scenario():
    """Analyze why PN isn't hitting the target."""

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

    print("=" * 80)
    print("SCENARIO ANALYSIS")
    print("=" * 80)

    # Calculate theoretical performance
    initial_distance = 8000.0
    missile_speed = 350.0
    target_speed = 200.0
    closing_speed = missile_speed - target_speed  # 150 m/s
    max_time = config.max_time

    theoretical_distance_covered = closing_speed * max_time
    theoretical_final_distance = initial_distance - theoretical_distance_covered

    print(f"\nInitial Setup:")
    print(f"  Initial Distance: {initial_distance} m")
    print(f"  Missile Velocity: {missile_speed} m/s")
    print(f"  Target Velocity: {target_speed} m/s")
    print(f"  Closing Speed: {closing_speed} m/s")
    print(f"  Max Time: {max_time} s")

    print(f"\nTheoretical Analysis (with PN, no lateral maneuvers needed):")
    print(f"  Distance can be covered in {max_time}s: {theoretical_distance_covered:.1f} m")
    print(f"  Theoretical final distance: {theoretical_final_distance:.1f} m")

    print(f"\nConclusion:")
    if theoretical_final_distance > config.proximity_threshold:
        print(f"  [EXPECTED MISS] Even with perfect PN, missile will be {theoretical_final_distance:.1f}m away")
        print(f"  Need {theoretical_final_distance / closing_speed:.1f}s more time to intercept")
        print(f"  OR need {(initial_distance / config.proximity_threshold - 1) * 100:.0f}% more closing speed")
    else:
        print(f"  [EXPECTED HIT] PN should intercept with {theoretical_final_distance:.1f}m margin")

    # Now run actual simulation
    print("\n" + "=" * 80)
    print("RUNNING SIMULATION")
    print("=" * 80)

    env = BallistXEnv(config=config)
    obs, info = env.reset()

    total_reward = 0
    for step in range(config.max_steps):
        baseline = info['baseline_accel']
        action = baseline / config.missile_max_accel

        obs, reward, term, trunc, info = env.step(action)
        total_reward += reward

        if term or trunc:
            print(f"\nTerminated at step {step} ({info['current_time']:.2f}s): {info['terminated_reason']}")
            print(f"Final distance: {info['distance']:.1f}m")
            print(f"Total reward: {total_reward:.1f}")
            break

    env.close()

if __name__ == '__main__':
    analyze_scenario()
