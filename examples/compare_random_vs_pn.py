"""
Random Agent vs PN Guidance Comparison

Compare pure random actions vs optimal Proportional Navigation baseline.
This shows how much room there is for RL to improve.
"""

import sys
from pathlib import Path

# Add ballistx_gym package to path
gym_path = Path(__file__).parent.parent
sys.path.insert(0, str(gym_path))

import numpy as np


def run_comparison(num_episodes=20):
    """
    Comprehensive comparison: Random vs PN Guidance
    """
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

    print("=" * 80)
    print(" " * 20 + "MISSILE GUIDANCE SHOWDOWN: RANDOM vs PROPORTIONAL NAVIGATION")
    print("=" * 80)

    # Calculate achievable scenario: missile should be able to intercept
    # For intercept: closing_speed * max_time > initial_distance
    # With 400m/s closing speed and 20s, we can cover 8000m (exactly!)
    config = BallistXConfig(
        dt=0.02,
        max_steps=1000,  # 20 seconds at dt=0.02
        max_time=20.0,
        missile_pos=(0.0, 5000.0, 0.0),
        missile_vel=(500.0, 0.0, 0.0),  # Fast missile
        missile_max_accel=400.0,
        target_pos=(8000.0, 5000.0, 0.0),
        target_vel=(100.0, 0.0, 0.0),  # Slow target
        proximity_threshold=15.0,
        miss_distance_threshold=20000.0,
        reward_miss_penalty=-100.0,
    )

    # ==================== RANDOM AGENT ====================
    print("\n" + "=" * 80)
    print("ROUND 1: RANDOM AGENT (Pure Chaos)")
    print("=" * 80)
    print("Actions: Completely random, no intelligence whatsoever\n")

    env_random = BallistXEnv(config=config)
    random_results = []

    for ep in range(1, num_episodes + 1):
        obs, info = env_random.reset()
        total_reward = 0
        initial_distance = info['distance']
        distances = [initial_distance]

        for step in range(1000):
            action = env_random.action_space.sample()
            obs, reward, term, trunc, info = env_random.step(action)
            total_reward += reward
            distances.append(info['distance'])

            if term or trunc:
                break

        random_results.append({
            'episode': ep,
            'reward': total_reward,
            'steps': step + 1,
            'final_distance': info['distance'],
            'distance_reduction': initial_distance - info['distance'],
            'reason': info['terminated_reason']
        })

        if ep <= 5 or ep == num_episodes:
            print(f"  Episode {ep:2d}: Reward={total_reward:8.1f}, Dist={distances[-1]:6.1f}m, Reduction={initial_distance - distances[-1]:6.1f}m")

    env_random.close()

    # ==================== PN GUIDANCE (OPTIMAL) ====================
    print("\n" + "=" * 80)
    print("ROUND 2: PROPORTIONAL NAVIGATION (Optimal Guidance)")
    print("=" * 80)
    print("Actions: Classical missile guidance law, battle-tested since 1960s\n")

    env_pn = BallistXEnv(config=config)
    pn_results = []

    for ep in range(1, num_episodes + 1):
        obs, info = env_pn.reset()
        total_reward = 0
        initial_distance = info['distance']
        distances = [initial_distance]

        for step in range(1000):
            # Use baseline PN guidance from info dict
            baseline = info['baseline_accel']
            action = baseline / config.missile_max_accel  # Normalize

            obs, reward, term, trunc, info = env_pn.step(action)
            total_reward += reward
            distances.append(info['distance'])

            if term or trunc:
                break

        pn_results.append({
            'episode': ep,
            'reward': total_reward,
            'steps': step + 1,
            'final_distance': info['distance'],
            'distance_reduction': initial_distance - info['distance'],
            'reason': info['terminated_reason']
        })

        if ep <= 5 or ep == num_episodes:
            print(f"  Episode {ep:2d}: Reward={total_reward:8.1f}, Dist={distances[-1]:6.1f}m, Reduction={initial_distance - distances[-1]:6.1f}m")

    env_pn.close()

    # ==================== STATISTICS ====================
    print("\n" + "=" * 80)
    print("FINAL SCORECARD")
    print("=" * 80)

    random_avg_reward = np.mean([r['reward'] for r in random_results])
    random_avg_dist = np.mean([r['final_distance'] for r in random_results])
    random_avg_reduction = np.mean([r['distance_reduction'] for r in random_results])

    pn_avg_reward = np.mean([r['reward'] for r in pn_results])
    pn_avg_dist = np.mean([r['final_distance'] for r in pn_results])
    pn_avg_reduction = np.mean([r['distance_reduction'] for r in pn_results])

    print(f"\n{'METRIC':<25} {'RANDOM AGENT':<20} {'PN GUIDANCE':<20} {'WINNER'}")
    print("-" * 80)
    print(f"{'Avg Reward':<25} {random_avg_reward:>10.1f}          {pn_avg_reward:>10.1f}          {'PN' if pn_avg_reward > random_avg_reward else 'RANDOM' if random_avg_reward > pn_avg_reward else 'TIE'}")
    print(f"{'Avg Final Distance (m)':<25} {random_avg_dist:>10.1f}          {pn_avg_dist:>10.1f}          {'PN' if pn_avg_dist < random_avg_dist else 'RANDOM' if random_avg_dist < pn_avg_dist else 'TIE'}")
    print(f"{'Avg Distance Reduction (m)':<25} {random_avg_reduction:>10.1f}          {pn_avg_reduction:>10.1f}          {'PN' if pn_avg_reduction > random_avg_reduction else 'RANDOM' if random_avg_reduction > pn_avg_reduction else 'TIE'}")

    # Calculate improvement percentage
    reward_improvement = ((pn_avg_reward - random_avg_reward) / abs(random_avg_reward)) * 100 if random_avg_reward != 0 else 0
    distance_improvement = ((random_avg_dist - pn_avg_dist) / random_avg_dist) * 100 if random_avg_dist != 0 else 0

    print("-" * 80)
    print(f"\n[PERFORMANCE GAP]")
    print(f"   Reward Improvement: {reward_improvement:+.1f}%")
    print(f"   Distance Improvement: {distance_improvement:+.1f}% (closer is better)")

    # ==================== EPISODE BY EPISODE ====================
    print("\n" + "=" * 80)
    print("EPISODE-BY-EPISODE BREAKDOWN")
    print("=" * 80)
    print(f"{'Ep':<4} {'Random Reward':<15} {'PN Reward':<15} {'Random Dist':<15} {'PN Dist':<15}")
    print("-" * 80)

    for i in range(num_episodes):
        rr = random_results[i]['reward']
        pr = pn_results[i]['reward']
        rd = random_results[i]['final_distance']
        pd = pn_results[i]['final_distance']

        winner = "PN" if pr > rr else "RANDOM" if rr > pr else "="
        print(f"{i+1:<4} {rr:>10.1f}         {pr:>10.1f}         {rd:>10.1f}m       {pd:>10.1f}m  {winner}")

    # ==================== CONCLUSION ====================
    print("\n" + "=" * 80)
    print("ANALYSIS")
    print("=" * 80)

    if pn_avg_reward > random_avg_reward:
        print(f"\n[WINNER] PROPORTIONAL NAVIGATION")
        print(f"   Outperforms random by {abs(reward_improvement):.1f}% in reward")
        print(f"   Gets {random_avg_dist - pn_avg_dist:.1f}m closer to target on average")
    else:
        print(f"\n[WINNER] RANDOM AGENT (Unexpected!)")
        print(f"   This shouldn't happen - check environment configuration")

    print(f"\n[KEY INSIGHT]")
    print(f"   The 'room for improvement' for your RL agent is: {abs(pn_avg_reward - random_avg_reward):.1f} reward points")
    print(f"   If your trained agent achieves > {pn_avg_reward:.0f} reward, it beats PN!")

    print("\n" + "=" * 80)

    return {
        'random': random_results,
        'pn': pn_results,
        'summary': {
            'random_avg_reward': random_avg_reward,
            'pn_avg_reward': pn_avg_reward,
            'random_avg_dist': random_avg_dist,
            'pn_avg_dist': pn_avg_dist,
            'reward_improvement_pct': reward_improvement,
        }
    }


def visualize_comparison(random_results, pn_results):
    """Create a simple text-based visualization."""
    print("\n" + "=" * 80)
    print("VISUAL COMPARISON (Final Distance Distribution)")
    print("=" * 80)

    random_dists = [r['final_distance'] for r in random_results]
    pn_dists = [r['final_distance'] for r in pn_results]

    bins = [0, 500, 1000, 2000, 5000, 10000, 20000]

    print("\nDistance Range    Random Agent    PN Guidance")
    print("-" * 50)

    for i in range(len(bins) - 1):
        r_count = sum(1 for d in random_dists if bins[i] <= d < bins[i+1])
        p_count = sum(1 for d in pn_dists if bins[i] <= d < bins[i+1])

        bar_r = "#" * r_count
        bar_p = "#" * p_count

        range_str = f"{bins[i]:>5}-{bins[i+1]:<6}m"
        print(f"{range_str}    {bar_r if bar_r else '0':<14}  {bar_p if bar_p else '0':<14}")

    print("\nLegend: Each # = 1 episode")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Compare Random vs PN Guidance')
    parser.add_argument('--episodes', type=int, default=20, help='Number of episodes')

    args = parser.parse_args()

    results = run_comparison(args.episodes)
    visualize_comparison(results['random'], results['pn'])
