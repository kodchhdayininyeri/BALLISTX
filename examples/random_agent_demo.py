"""
Random Agent Demo for BALLISTX Gymnasium Environment

A simple demonstration of running a random agent in the missile guidance environment.
"""

import sys
from pathlib import Path

# Add ballistx_gym package to path
gym_path = Path(__file__).parent.parent
sys.path.insert(0, str(gym_path))

import numpy as np


def run_random_agent(num_episodes=5, max_steps_per_episode=500, render=False):
    """
    Run a random agent in the BALLISTX environment.

    Args:
        num_episodes: Number of episodes to run
        max_steps_per_episode: Maximum steps per episode
        render: Whether to render the environment
    """
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

    # Configure environment
    config = BallistXConfig(
        dt=0.02,
        max_steps=max_steps_per_episode,
        max_time=30.0,
        missile_pos=(0.0, 5000.0, 0.0),
        missile_vel=(350.0, 0.0, 0.0),
        missile_max_accel=400.0,
        target_pos=(8000.0, 5000.0, 0.0),
        target_vel=(200.0, 0.0, 0.0),
        proximity_threshold=15.0,
        miss_distance_threshold=15000.0,  # More tolerant for random agent
        reward_miss_penalty=-100.0,  # Reduce penalty
    )

    render_mode = 'human' if render else None
    env = BallistXEnv(config=config, render_mode=render_mode)

    print("=" * 70)
    print("RANDOM AGENT DEMO - BALLISTX MISSILE GUIDANCE")
    print("=" * 70)
    print(f"\nConfiguration:")
    print(f"  Missile Start: {config.missile_pos} @ {config.missile_vel} m/s")
    print(f"  Target Start:  {config.target_pos} @ {config.target_vel} m/s")
    print(f"  Max Acceleration: {config.missile_max_accel} m/s²")
    print(f"  Hit Threshold: {config.proximity_threshold}m")
    print(f"  Episodes: {num_episodes}")
    print()

    episode_results = []

    for episode in range(1, num_episodes + 1):
        obs, info = env.reset()
        episode_reward = 0.0
        episode_steps = 0
        distances = [info['distance']]

        print(f"Episode {episode}/{num_episodes}")
        print(f"  Initial: Distance = {info['distance']:.1f}m")

        while episode_steps < max_steps_per_episode:
            # Sample random action
            action = env.action_space.sample()

            # Add bias toward target (simplified guidance)
            # Get baseline PN guidance from info
            if episode_steps == 0:
                baseline = info['baseline_accel']
                baseline_action = baseline / config.missile_max_accel
            else:
                # Keep using the same baseline (simplified)
                pass

            # Mix random action with guidance bias (30% guided, 70% random)
            action = action * 0.7 + baseline_action * 0.3

            # Scale down overall
            action = action * 0.3  # Use 30% of mixed action

            # Step environment
            obs, reward, terminated, truncated, info = env.step(action)

            episode_reward += reward
            episode_steps += 1
            distances.append(info['distance'])

            if terminated or truncated:
                break

        # Episode results
        result = {
            'episode': episode,
            'reward': episode_reward,
            'steps': episode_steps,
            'final_distance': distances[-1],
            'distance_reduction': distances[0] - distances[-1],
            'reason': info.get('terminated_reason', 'unknown'),
            'hit': info.get('is_hit', False)
        }
        episode_results.append(result)

        # Print episode summary
        print(f"  Result: {result['reason'].upper()}")
        print(f"  Steps: {result['steps']}")
        print(f"  Final Distance: {result['final_distance']:.1f}m")
        print(f"  Distance Reduction: {result['distance_reduction']:.1f}m")
        print(f"  Total Reward: {result['reward']:.1f}")
        print(f"  {'*** HIT! ***' if result['hit'] else ''}")
        print()

    env.close()

    # Overall statistics
    print("=" * 70)
    print("OVERALL STATISTICS")
    print("=" * 70)

    hits = sum(1 for r in episode_results if r['hit'])
    avg_reward = np.mean([r['reward'] for r in episode_results])
    avg_steps = np.mean([r['steps'] for r in episode_results])
    avg_reduction = np.mean([r['distance_reduction'] for r in episode_results])

    print(f"  Episodes: {num_episodes}")
    print(f"  Hits: {hits}/{num_episodes} ({100*hits/num_episodes:.1f}%)")
    print(f"  Avg Reward: {avg_reward:.1f}")
    print(f"  Avg Steps: {avg_steps:.1f}")
    print(f"  Avg Distance Reduction: {avg_reduction:.1f}m")
    print("=" * 70)

    return episode_results


def run_comparison_agent(num_episodes=3):
    """
    Compare random actions vs baseline PN guidance.
    """
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

    config = BallistXConfig(
        dt=0.02,
        max_steps=500,
        missile_pos=(0.0, 5000.0, 0.0),
        missile_vel=(350.0, 0.0, 0.0),
        missile_max_accel=400.0,
        target_pos=(8000.0, 5000.0, 0.0),
        target_vel=(200.0, 0.0, 0.0),
        miss_distance_threshold=15000.0,
        reward_miss_penalty=-100.0,
    )

    print("=" * 70)
    print("COMPARISON: Random Agent vs Proportional Navigation")
    print("=" * 70)

    # Test Random Agent
    print("\n--- Random Agent ---")
    env = BallistXEnv(config=config)
    random_results = []

    for ep in range(num_episodes):
        obs, info = env.reset()
        total_reward = 0

        for step in range(500):
            action = env.action_space.sample() * 0.3  # 30% random
            obs, reward, term, trunc, info = env.step(action)
            total_reward += reward

            if term or trunc:
                random_results.append({
                    'reward': total_reward,
                    'final_distance': info['distance'],
                    'reason': info['terminated_reason']
                })
                break

    env.close()

    # Print random results
    print(f"Random Agent Results ({num_episodes} episodes):")
    print(f"  Avg Reward: {np.mean([r['reward'] for r in random_results]):.1f}")
    print(f"  Avg Final Distance: {np.mean([r['final_distance'] for r in random_results]):.1f}m")

    # Test PN Baseline (from info dict)
    print("\n--- Proportional Navigation Baseline ---")
    env = BallistXEnv(config=config)
    pn_results = []

    for ep in range(num_episodes):
        obs, info = env.reset()
        total_reward = 0

        for step in range(500):
            # Use baseline PN guidance from info
            baseline = info['baseline_accel']
            action = baseline / config.missile_max_accel  # Normalize to [-1, 1]

            obs, reward, term, trunc, info = env.step(action)
            total_reward += reward

            if term or trunc:
                pn_results.append({
                    'reward': total_reward,
                    'final_distance': info['distance'],
                    'reason': info['terminated_reason']
                })
                break

    env.close()

    # Print PN results
    print(f"PN Guidance Results ({num_episodes} episodes):")
    print(f"  Avg Reward: {np.mean([r['reward'] for r in pn_results]):.1f}")
    print(f"  Avg Final Distance: {np.mean([r['final_distance'] for r in pn_results]):.1f}m")

    print("\n" + "=" * 70)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Run random agent in BALLISTX environment')
    parser.add_argument('--episodes', type=int, default=10, help='Number of episodes')
    parser.add_argument('--render', action='store_true', help='Render environment')
    parser.add_argument('--compare', action='store_true', help='Run comparison with PN baseline')

    args = parser.parse_args()

    if args.compare:
        run_comparison_agent(num_episodes=5)
    else:
        run_random_agent(num_episodes=args.episodes, render=args.render)
