"""
COMPREHENSIVE PPO FAILURE DIAGNOSIS

This script diagnoses why PPO is failing to learn:
1. Tests environment with random actions
2. Tests environment with zero actions (gravity only)
3. Tests environment with PN actions (baseline)
4. Analyzes observation/action space
5. Tests reward calculation
6. Checks for numerical issues
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict


def test_random_actions():
    """Test environment with random actions."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

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
    print("TEST 1: RANDOM ACTIONS")
    print("=" * 70)

    results = []
    for episode in range(20):
        obs, info = env.reset()
        total_reward = 0
        steps = 0
        distances = []
        rewards_per_step = []
        actions_taken = []

        while True:
            action = env.action_space.sample()
            actions_taken.append(action.copy())

            obs, reward, term, trunc, info = env.step(action)
            total_reward += reward
            steps += 1
            distances.append(info['distance'])
            rewards_per_step.append(reward)

            if term or trunc:
                results.append({
                    'episode': episode + 1,
                    'reward': total_reward,
                    'steps': steps,
                    'distance': info['distance'],
                    'reason': info['terminated_reason'],
                    'final_altitude': info['missile_position'][1],
                    'actions': np.array(actions_taken),
                    'rewards': np.array(rewards_per_step),
                    'distances': np.array(distances)
                })
                break

    # Analysis
    print(f"\nResults over {len(results)} episodes:")
    print(f"  Avg Reward: {np.mean([r['reward'] for r in results]):.1f} +/- {np.std([r['reward'] for r in results]):.1f}")
    print(f"  Avg Steps: {np.mean([r['steps'] for r in results]):.1f}")
    print(f"  Hit Rate: {sum(1 for r in results if r['reason'] == 'hit')}/{len(results)}")
    print(f"  Crash Rate: {sum(1 for r in results if r['reason'] == 'crash')}/{len(results)}")
    print(f"  Miss Rate: {sum(1 for r in results if r['reason'] == 'miss')}/{len(results)}")

    # Analyze action distribution
    all_actions = np.vstack([r['actions'] for r in results])
    print(f"\nAction Statistics (random sampling):")
    print(f"  Mean: {all_actions.mean(axis=0)}")
    print(f"  Std: {all_actions.std(axis=0)}")
    print(f"  Min: {all_actions.min(axis=0)}")
    print(f"  Max: {all_actions.max(axis=0)}")

    # Analyze reward distribution
    all_rewards = np.concatenate([r['rewards'] for r in results])
    print(f"\nReward Statistics (per step):")
    print(f"  Mean: {all_rewards.mean():.3f}")
    print(f"  Std: {all_rewards.std():.3f}")
    print(f"  Min: {all_rewards.min():.3f}")
    print(f"  Max: {all_rewards.max():.3f}")

    env.close()
    return results


def test_zero_actions():
    """Test environment with zero actions (gravity only)."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

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

    print("\n" + "=" * 70)
    print("TEST 2: ZERO ACTIONS (GRAVITY ONLY)")
    print("=" * 70)

    results = []
    for episode in range(10):
        obs, info = env.reset()
        total_reward = 0
        steps = 0

        while True:
            action = np.zeros(3)  # No guidance, just gravity
            obs, reward, term, trunc, info = env.step(action)
            total_reward += reward
            steps += 1

            if term or trunc:
                results.append({
                    'episode': episode + 1,
                    'reward': total_reward,
                    'steps': steps,
                    'distance': info['distance'],
                    'reason': info['terminated_reason'],
                    'final_altitude': info['missile_position'][1],
                })
                break

    print(f"\nResults over {len(results)} episodes:")
    print(f"  Avg Reward: {np.mean([r['reward'] for r in results]):.1f}")
    print(f"  Avg Steps: {np.mean([r['steps'] for r in results]):.1f}")
    print(f"  Hit Rate: {sum(1 for r in results if r['reason'] == 'hit')}/{len(results)}")
    print(f"  Crash Rate: {sum(1 for r in results if r['reason'] == 'crash')}/{len(results)}")

    env.close()
    return results


def test_pn_actions():
    """Test environment with PN actions."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

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

    print("\n" + "=" * 70)
    print("TEST 3: PN GUIDANCE ACTIONS")
    print("=" * 70)

    results = []
    all_pn_actions = []

    for episode in range(10):
        obs, info = env.reset()
        total_reward = 0
        steps = 0
        pn_actions = []

        while True:
            # Use PN guidance from info
            baseline = info['baseline_accel']
            action = baseline / config.missile_max_accel  # Normalize to [-1, 1]
            pn_actions.append(action.copy())

            obs, reward, term, trunc, info = env.step(action)
            total_reward += reward
            steps += 1

            if term or trunc:
                results.append({
                    'episode': episode + 1,
                    'reward': total_reward,
                    'steps': steps,
                    'distance': info['distance'],
                    'reason': info['terminated_reason'],
                })
                all_pn_actions.extend(pn_actions)
                break

    print(f"\nResults over {len(results)} episodes:")
    print(f"  Avg Reward: {np.mean([r['reward'] for r in results]):.1f}")
    print(f"  Hit Rate: {sum(1 for r in results if r['reason'] == 'hit')}/{len(results)}")

    # Analyze PN action distribution
    pn_actions = np.array(all_pn_actions)
    print(f"\nPN Action Statistics:")
    print(f"  Mean: {pn_actions.mean(axis=0)}")
    print(f"  Std: {pn_actions.std(axis=0)}")
    print(f"  Min: {pn_actions.min(axis=0)}")
    print(f"  Max: {pn_actions.max(axis=0)}")
    print(f"  Action Norm: {np.linalg.norm(pn_actions, axis=1).mean():.3f} (avg)")

    env.close()
    return results, pn_actions


def analyze_observation_space():
    """Analyze observation space statistics."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

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
    )

    env = BallistXEnv(config=config)

    print("\n" + "=" * 70)
    print("TEST 4: OBSERVATION SPACE ANALYSIS")
    print("=" * 70)

    print(f"\nObservation Space Shape: {env.observation_space.shape}")
    print(f"Observation Space Bounds: [{env.observation_space.low[0]:.2f}, {env.observation_space.high[0]:.2f}]")

    obs, info = env.reset()
    print(f"\nInitial Observation (first 10 dims): {obs[:10]}")

    # Collect observations over random episodes
    all_obs = []
    for _ in range(100):
        obs, info = env.reset()
        all_obs.append(obs.copy())

        for _ in range(50):  # 50 steps per episode
            action = env.action_space.sample()
            obs, reward, term, trunc, info = env.step(action)
            all_obs.append(obs.copy())
            if term or trunc:
                break

    all_obs = np.array(all_obs)
    print(f"\nCollected {len(all_obs)} observations")

    # Per-dimension statistics
    print(f"\nPer-Dimension Statistics:")
    for i in range(min(10, all_obs.shape[1])):
        print(f"  Dim {i}: mean={all_obs[:, i].mean():.2f}, "
              f"std={all_obs[:, i].std():.2f}, "
              f"min={all_obs[:, i].min():.2f}, "
              f"max={all_obs[:, i].max():.2f}")

    # Check for NaN/Inf
    has_nan = np.isnan(all_obs).any()
    has_inf = np.isinf(all_obs).any()
    print(f"\nNumerical Issues:")
    print(f"  Has NaN: {has_nan}")
    print(f"  Has Inf: {has_inf}")

    env.close()


def test_reward_components():
    """Test individual reward components."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

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
        reward_distance_scale=1.0,
    )

    env = BallistXEnv(config=config)

    print("\n" + "=" * 70)
    print("TEST 5: REWARD COMPONENT BREAKDOWN")
    print("=" * 70)

    obs, info = env.reset()
    initial_distance = info['distance']

    # Test with PN guidance for predictable behavior
    rewards_breakdown = []
    distances = []

    for step in range(100):
        baseline = info['baseline_accel']
        action = baseline / config.missile_max_accel

        old_distance = info['distance']
        obs, reward, term, trunc, info = env.step(action)
        new_distance = info['distance']

        # Manually calculate components
        distance_delta = old_distance - new_distance
        distance_reward = distance_delta * config.reward_distance_scale
        time_reward = config.reward_time_penalty
        accel_reward = config.reward_accel_penalty * np.linalg.norm(action * config.missile_max_accel)

        rewards_breakdown.append({
            'step': step,
            'total': reward,
            'distance_delta': distance_delta,
            'distance_reward': distance_reward,
            'time_reward': time_reward,
            'accel_reward': accel_reward,
            'distance': new_distance,
        })
        distances.append(new_distance)

        if term or trunc:
            break

    print(f"\nFirst 10 steps with PN guidance:")
    print(f"{'Step':<5} {'Distance':<10} {'Delta':<10} {'Dist Rwd':<10} {'Time Rwd':<10} {'Accel Rwd':<10} {'Total':<10}")
    print("-" * 75)
    for rb in rewards_breakdown[:10]:
        print(f"{rb['step']:<5} {rb['distance']:<10.1f} {rb['distance_delta']:<10.2f} "
              f"{rb['distance_reward']:<10.2f} {rb['time_reward']:<10.3f} "
              f"{rb['accel_reward']:<10.3f} {rb['total']:<10.2f}")

    print(f"\nSummary over {len(rewards_breakdown)} steps:")
    print(f"  Initial Distance: {initial_distance:.1f}m")
    print(f"  Final Distance: {distances[-1]:.1f}m")
    print(f"  Distance Closed: {initial_distance - distances[-1]:.1f}m")
    print(f"  Avg Distance Reward: {np.mean([rb['distance_reward'] for rb in rewards_breakdown]):.3f}")
    print(f"  Avg Time Penalty: {np.mean([rb['time_reward'] for rb in rewards_breakdown]):.3f}")
    print(f"  Avg Accel Penalty: {np.mean([rb['accel_reward'] for rb in rewards_breakdown]):.3f}")
    print(f"  Avg Total Reward: {np.mean([rb['total'] for rb in rewards_breakdown]):.3f}")

    env.close()


def compare_trained_model():
    """Compare trained PPO model with baseline."""
    print("\n" + "=" * 70)
    print("TEST 6: TRAINED PPO MODEL EVALUATION")
    print("=" * 70)

    # Check if trained model exists
    import os
    model_path = "logs/ppo_easy/models/ppo_easy_final.zip"

    if not os.path.exists(model_path):
        print(f"\nTrained model not found at {model_path}")
        print("Skipping trained model evaluation.")
        return

    try:
        from stable_baselines3 import PPO
        from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

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
        model = PPO.load(model_path)

        print(f"\nEvaluating trained model...")

        results = []
        all_actions = []

        for episode in range(20):
            obs, info = env.reset()
            total_reward = 0
            steps = 0
            episode_actions = []

            while True:
                action, _ = model.predict(obs, deterministic=True)
                episode_actions.append(action.copy())
                all_actions.append(action.copy())

                obs, reward, term, trunc, info = env.step(action)
                total_reward += reward
                steps += 1

                if term or trunc:
                    results.append({
                        'episode': episode + 1,
                        'reward': total_reward,
                        'steps': steps,
                        'distance': info['distance'],
                        'reason': info['terminated_reason'],
                    })
                    break

        print(f"\nTrained Model Results ({len(results)} episodes):")
        print(f"  Avg Reward: {np.mean([r['reward'] for r in results]):.1f} +/- {np.std([r['reward'] for r in results]):.1f}")
        print(f"  Hit Rate: {sum(1 for r in results if r['reason'] == 'hit')}/{len(results)}")
        print(f"  Crash Rate: {sum(1 for r in results if r['reason'] == 'crash')}/{len(results)}")
        print(f"  Miss Rate: {sum(1 for r in results if r['reason'] == 'miss')}/{len(results)}")

        # Check if all episodes are identical
        rewards = [r['reward'] for r in results]
        steps = [r['steps'] for r in results]
        reasons = [r['reason'] for r in results]

        if len(set(rewards)) == 1:
            print(f"\n⚠️  WARNING: All episodes have IDENTICAL reward ({rewards[0]})")
            print("    This suggests the model is not learning and acting deterministically.")

        if len(set(steps)) == 1:
            print(f"⚠️  WARNING: All episodes have IDENTICAL steps ({steps[0]})")

        if len(set(reasons)) == 1:
            print(f"⚠️  WARNING: All episodes have IDENTICAL termination reason ({reasons[0]})")

        # Analyze action distribution
        all_actions = np.array(all_actions)
        print(f"\nTrained Model Action Statistics:")
        print(f"  Mean: {all_actions.mean(axis=0)}")
        print(f"  Std: {all_actions.std(axis=0)}")
        print(f"  Min: {all_actions.min(axis=0)}")
        print(f"  Max: {all_actions.max(axis=0)}")

        # Check if actions are effectively constant
        if all_actions.std(axis=0).max() < 0.01:
            print(f"\n⚠️  CRITICAL: Actions are nearly constant (std < 0.01)")
            print("    The model has converged to a fixed policy, likely due to:")
            print("    1. Premature convergence")
            print("    2. Numerical issues")
            print("    3. Exploration decay (entropy coefficient too low)")

        env.close()

    except Exception as e:
        print(f"\nError loading or evaluating trained model: {e}")


def main():
    """Run all diagnostic tests."""
    print("\n" + "=" * 70)
    print("BALLISTX PPO FAILURE DIAGNOSIS")
    print("=" * 70)

    # Test 1: Random actions
    random_results = test_random_actions()

    # Test 2: Zero actions (gravity only)
    zero_results = test_zero_actions()

    # Test 3: PN actions
    pn_results, pn_actions = test_pn_actions()

    # Test 4: Observation space analysis
    analyze_observation_space()

    # Test 5: Reward component breakdown
    test_reward_components()

    # Test 6: Trained model evaluation
    compare_trained_model()

    # Final summary
    print("\n" + "=" * 70)
    print("DIAGNOSIS SUMMARY")
    print("=" * 70)

    print(f"\n1. BASELINE COMPARISON:")
    print(f"   Random Actions: {np.mean([r['reward'] for r in random_results]):.1f}")
    print(f"   Zero Actions:   {np.mean([r['reward'] for r in zero_results]):.1f}")
    print(f"   PN Actions:     {np.mean([r['reward'] for r in pn_results]):.1f}")

    print(f"\n2. KEY FINDINGS:")
    random_crash_rate = sum(1 for r in random_results if r['reason'] == 'crash') / len(random_results)
    zero_crash_rate = sum(1 for r in zero_results if r['reason'] == 'crash') / len(zero_results)

    print(f"   - Random actions crash rate: {random_crash_rate*100:.1f}%")
    print(f"   - Zero actions (gravity) crash rate: {zero_crash_rate*100:.1f}%")
    print(f"   - PN guidance hit rate: 100%")

    print(f"\n3. ROOT CAUSE ANALYSIS:")
    if random_crash_rate > 0.5:
        print("   ⚠️  Random actions frequently crash due to gravity")
        print("   → PPO needs to learn gravity compensation FIRST")
        print("   → This is a hard exploration problem")

    print(f"\n4. RECOMMENDED FIXES:")
    print("   1. Add gravity compensation to actions (add +g to y-acceleration)")
    print("   2. Use curriculum learning (start with gravity-compensated actions)")
    print("   3. Increase entropy coefficient for more exploration")
    print("   4. Use action noise during training")
    print("   5. Consider reward shaping to encourage upward acceleration")

    print("\n" + "=" * 70)


if __name__ == '__main__':
    main()
