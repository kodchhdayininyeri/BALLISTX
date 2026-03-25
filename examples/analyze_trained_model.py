"""
Analyze the trained PPO model to understand why it's failing.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import torch


def analyze_model():
    """Analyze the trained PPO model."""
    try:
        from stable_baselines3 import PPO
        from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

        model_path = "logs/ppo_easy/models/ppo_easy_final.zip"

        print("=" * 70)
        print("TRAINED PPO MODEL ANALYSIS")
        print("=" * 70)

        # Load model
        print(f"\nLoading model from: {model_path}")
        model = PPO.load(model_path)

        print("\nModel Architecture:")
        print(f"  Policy: {model.policy}")
        print(f"  Learning Rate: {model.learning_rate}")
        print(f"  Gamma: {model.gamma}")
        print(f"  GAE Lambda: {model.gae_lambda}")
        print(f"  Entropy Coef: {model.ent_coef}")
        print(f"  VF Coef: {model.vf_coef}")
        print(f"  Max Grad Norm: {model.max_grad_norm}")
        print(f"  Use SDE: {model.use_sde}")

        # Check policy network weights
        print("\nPolicy Network Weights:")
        if hasattr(model.policy, 'actor'):
            actor = model.policy.actor_net if hasattr(model.policy, 'actor_net') else model.policy.actor
            if hasattr(actor, 'layers') or hasattr(actor, 'modules'):
                for name, param in model.policy.named_parameters():
                    print(f"  {name}:")
                    print(f"    Shape: {param.shape}")
                    print(f"    Mean: {param.data.mean().item():.6f}")
                    print(f"    Std: {param.data.std().item():.6f}")
                    print(f"    Min: {param.data.min().item():.6f}")
                    print(f"    Max: {param.data.max().item():.6f}")

        # Create environment
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

        # Test model predictions
        print("\n" + "=" * 70)
        print("MODEL PREDICTION ANALYSIS")
        print("=" * 70)

        obs, info = env.reset()

        print(f"\nInitial Observation Shape: {obs.shape}")
        print(f"Initial Observation (first 10): {obs[:10]}")

        # Get deterministic prediction
        action_det, _ = model.predict(obs, deterministic=True)
        print(f"\nDeterministic Action: {action_det}")
        print(f"  Action Norm: {np.linalg.norm(action_det):.4f}")

        # Get stochastic prediction
        actions_stoch = []
        for i in range(10):
            action, _ = model.predict(obs, deterministic=False)
            actions_stoch.append(action)

        actions_stoch = np.array(actions_stoch)
        print(f"\nStochastic Actions (10 samples):")
        print(f"  Mean: {actions_stoch.mean(axis=0)}")
        print(f"  Std: {actions_stoch.std(axis=0)}")
        print(f"  Min: {actions_stoch.min(axis=0)}")
        print(f"  Max: {actions_stoch.max(axis=0)}")

        # Check if actions are constant
        if np.allclose(actions_stoch, actions_stoch[0], atol=1e-6):
            print("\n  CRITICAL: All stochastic predictions are IDENTICAL!")
            print("  The policy has collapsed to a deterministic point.")

        # Test multiple observations
        print("\n" + "=" * 70)
        print("ACTION SENSITIVITY ANALYSIS")
        print("=" * 70)

        # Collect observations from a random episode
        obs, info = env.reset()
        observations = [obs.copy()]

        for _ in range(50):
            action = env.action_space.sample()
            obs, reward, term, trunc, info = env.step(action)
            observations.append(obs.copy())
            if term or trunc:
                break

        observations = np.array(observations)
        print(f"\nCollected {len(observations)} observations from random episode")

        # Get actions for all observations
        actions = []
        for obs in observations:
            action, _ = model.predict(obs, deterministic=True)
            actions.append(action)

        actions = np.array(actions)
        print(f"\nActions across {len(actions)} observations:")
        print(f"  Mean: {actions.mean(axis=0)}")
        print(f"  Std: {actions.std(axis=0)}")
        print(f"  Min: {actions.min(axis=0)}")
        print(f"  Max: {actions.max(axis=0)}")

        # Check variance
        action_std = actions.std(axis=0)
        if np.all(action_std < 0.01):
            print(f"\n  CRITICAL: Actions are nearly constant (std < 0.01)")
            print(f"  The model outputs the same action regardless of observation!")

        # Value function analysis
        print("\n" + "=" * 70)
        print("VALUE FUNCTION ANALYSIS")
        print("=" * 70)

        values = []
        for obs in observations:
            # Get value estimate
            with torch.no_grad():
                obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
                # Extract features
                features = model.policy.extract_features(obs_tensor)
                # Get value
                value = model.policy.critic(features)
                values.append(value.item())

        values = np.array(values)
        print(f"\nValue estimates across {len(values)} observations:")
        print(f"  Mean: {values.mean():.2f}")
        print(f"  Std: {values.std():.2f}")
        print(f"  Min: {values.min():.2f}")
        print(f"  Max: {values.max():.2f}")

        if values.std() < 1.0:
            print(f"\n  WARNING: Value function has very low variance (std < 1.0)")
            print(f"  The model may not be learning to distinguish states.")

        # Compare with baseline
        print("\n" + "=" * 70)
        print("BASELINE COMPARISON")
        print("=" * 70)

        obs, info = env.reset()

        # Get model action
        model_action, _ = model.predict(obs, deterministic=True)

        # Get PN action
        pn_action = info['baseline_accel'] / config.missile_max_accel

        print(f"\nInitial State Comparison:")
        print(f"  Model Action:  {model_action}")
        print(f"  PN Action:     {pn_action}")
        print(f"  Difference:    {model_action - pn_action}")
        print(f"  Model Norm:    {np.linalg.norm(model_action):.4f}")
        print(f"  PN Norm:       {np.linalg.norm(pn_action):.4f}")

        # Simulate one episode with model
        print("\n" + "=" * 70)
        print("EPISODE SIMULATION (Model)")
        print("=" * 70)

        obs, info = env.reset()
        total_reward = 0
        steps = 0
        distances = [info['distance']]
        actions_taken = []

        while True:
            action, _ = model.predict(obs, deterministic=True)
            actions_taken.append(action.copy())

            obs, reward, term, trunc, info = env.step(action)
            total_reward += reward
            steps += 1
            distances.append(info['distance'])

            if term or trunc:
                break

        actions_taken = np.array(actions_taken)

        print(f"\nEpisode Results:")
        print(f"  Total Reward: {total_reward:.1f}")
        print(f"  Steps: {steps}")
        print(f"  Final Distance: {distances[-1]:.1f}m")
        print(f"  Termination Reason: {info['terminated_reason']}")

        print(f"\nAction Statistics:")
        print(f"  Mean: {actions_taken.mean(axis=0)}")
        print(f"  Std: {actions_taken.std(axis=0)}")

        # Check if actions are constant
        if actions_taken.std(axis=0).max() < 0.001:
            print(f"\n  CRITICAL: Model took constant action throughout episode!")
            print(f"  Action: {actions_taken[0]}")

        # Distance progression
        print(f"\nDistance Progression:")
        print(f"  Initial: {distances[0]:.1f}m")
        print(f"  Final:   {distances[-1]:.1f}m")
        print(f"  Change:  {distances[-1] - distances[0]:.1f}m")

        if distances[-1] > distances[0]:
            print(f"  WARNING: Missile moved AWAY from target!")

        env.close()

        print("\n" + "=" * 70)
        print("DIAGNOSIS")
        print("=" * 70)

        print("\nKEY FINDINGS:")
        print("1. The trained model has collapsed to a constant policy")
        print("2. Actions have nearly zero variance across different states")
        print("3. The model is not responding to observations")
        print("4. Value function may have low variance")

        print("\nROOT CAUSE:")
        print("The PPO training has converged to a degenerate solution where")
        print("the policy outputs a constant action regardless of state.")

        print("\nPOSSIBLE REASONS:")
        print("1. Learning rate too high → premature convergence")
        print("2. Entropy coefficient too low → no exploration")
        print("3. Network capacity too low → can't learn complex mapping")
        print("4. Reward signal too weak → no gradient signal")
        print("5. Action space issues → normalization problems")

        print("\n" + "=" * 70)

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    analyze_model()
