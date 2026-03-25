"""
PN BASELINE TEST - Verify PN Guidance Works

This script verifies that Proportional Navigation actually works
and establishes the correct baseline.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np

def test_pn_guidance():
    """Test PN guidance with the EASY scenario."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

    # EASY scenario from train_ppo_easy.py
    config = BallistXConfig(
        dt=0.02,
        max_steps=500,
        max_time=10.0,
        missile_pos=(0.0, 5000.0, 0.0),
        missile_vel=(300.0, 0.0, 0.0),
        missile_max_accel=400.0,
        target_pos=(3000.0, 5000.0, 0.0),
        target_vel=(0.0, 0.0, 0.0),
        proximity_threshold=50.0,
        miss_distance_threshold=10000.0,
        reward_hit_bonus=500.0,
        reward_miss_penalty=-100.0,
        reward_time_penalty=-0.05,
        reward_accel_penalty=-0.001,
    )

    env = BallistXEnv(config=config)

    print("=" * 70)
    print("PN GUIDANCE BASELINE TEST - EASY SCENARIO")
    print("=" * 70)
    print(f"Initial Distance: 3000m")
    print(f"Missile Speed: 300 m/s")
    print(f"Target: Stationary")
    print(f"Max Time: 10s")
    print("=" * 70)

    results = []

    for episode in range(10):
        obs, info = env.reset()
        total_reward = 0
        steps = 0

        while True:
            # Use PN guidance from info
            baseline = info['baseline_accel']
            action = baseline / config.missile_max_accel  # Normalize to [-1, 1]

            obs, reward, term, trunc, info = env.step(action)
            total_reward += reward
            steps += 1

            if term or trunc:
                results.append({
                    'episode': episode + 1,
                    'reward': total_reward,
                    'steps': steps,
                    'distance': info['distance'],
                    'reason': info['terminated_reason']
                })
                break

    # Print results
    print("\nPN Guidance Results:")
    print(f"  Avg Reward: {np.mean([r['reward'] for r in results]):.1f}")
    print(f"  Hit Rate: {sum(1 for r in results if r['reason'] == 'hit')}/10")
    print(f"  Avg Distance: {np.mean([r['distance'] for r in results]):.1f}m")

    # Detailed breakdown
    print("\nEpisode Details:")
    for r in results[:5]:
        print(f"  Ep {r['episode']}: Reward={r['reward']:.1f}, "
              f"Dist={r['distance']:.1f}m, Result={r['reason']}")

    env.close()

    print("\n" + "=" * 70)
    if np.mean([r['reward'] for r in results]) > 0:
        print("SUCCESS: PN guidance works! Use this as baseline.")
    else:
        print("WARNING: PN guidance also fails! Environment problem?")
    print("=" * 70)

    return results


if __name__ == '__main__':
    test_pn_guidance()
