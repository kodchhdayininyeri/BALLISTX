"""
PN + RL Hybrid Training

The agent learns to IMPROVE upon PN guidance rather than learning from scratch.
This is much easier because:
1. PN already provides good baseline actions
2. Agent only needs to learn small corrections
3. Much less exploration needed
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import os

try:
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
    from stable_baselines3.common.monitor import Monitor
except ImportError:
    print("Required packages not found.")
    sys.exit(1)


class HybridBallistXEnv:
    """
    Wrapper environment that provides PN guidance as baseline.
    The RL agent learns CORRECTIONS to PN, not full actions.
    """

    def __init__(self):
        from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

        self.config = BallistXConfig(
            dt=0.02,
            max_steps=1000,
            max_time=20.0,
            missile_pos=(0.0, 5000.0, 0.0),
            missile_vel=(500.0, 0.0, 0.0),
            missile_max_accel=400.0,
            target_pos=(8000.0, 5000.0, 0.0),
            target_vel=(100.0, 0.0, 0.0),
            proximity_threshold=15.0,
            miss_distance_threshold=20000.0,
        )

        self.env = BallistXEnv(config=self.config)
        self.action_space = self.env.action_space
        self.observation_space = self.env.observation_space

        # Track if we've loaded the real environment
        self._real_env = self.env

    def reset(self, seed=None, options=None):
        obs, info = self.env.reset(seed=seed, options=options)
        return obs, info

    def step(self, correction):
        """
        Apply RL correction ON TOP of PN guidance.

        Args:
            correction: RL agent's correction to PN action [-1, 1]

        Returns:
            Same as env.step()
        """
        # Get PN baseline action
        pn_action = self._get_pn_action()

        # Combine: final_action = pn_action + correction * correction_scale
        correction_scale = 0.3  # Allow 30% deviation from PN
        final_action = np.clip(pn_action + correction * correction_scale, -1.0, 1.0)

        return self.env.step(final_action)

    def _get_pn_action(self):
        """Get PN guidance baseline action."""
        info = self._get_info()
        pn_accel = info['baseline_accel']
        pn_action = pn_accel / self.config.missile_max_accel
        return np.clip(pn_action, -1.0, 1.0)

    def _get_info(self):
        """Get current info dict."""
        # We need to get info without stepping - create a dummy observation
        return self.env._get_info()

    def __getattr__(self, name):
        """Delegate all other attributes to wrapped env."""
        return getattr(self.env, name)


def create_hybrid_env():
    """Create hybrid environment."""
    return HybridBallistXEnv()


def train_hybrid_ppo(
    total_timesteps=200000,
    n_envs=4,
    log_dir='logs/ppo_hybrid'
):
    """Train PPO with PN baseline."""

    print("=" * 70)
    print("HYBRID TRAINING: PN + RL")
    print("=" * 70)
    print("Agent learns CORRECTIONS to PN guidance")
    print(f"Total Timesteps: {total_timesteps:,}")
    print(f"Log Directory: {log_dir}")
    print("=" * 70)

    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(f"{log_dir}/models", exist_ok=True)

    # Create vectorized environments
    def make_monitored_env():
        env = create_hybrid_env()
        # Wrap the actual env for monitoring
        return Monitor(env._real_env, log_dir, allow_early_resets=True)

    env = make_vec_env(make_monitored_env, n_envs=n_envs)

    print("\nCreating PPO model...")
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=1e-4,  # Conservative learning rate
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.15,  # Smaller clip for fine-tuning
        clip_range_vf=None,
        normalize_advantage=True,
        ent_coef=0.005,  # Low exploration - we mainly follow PN
        vf_coef=0.5,
        max_grad_norm=0.5,
        use_sde=False,  # No SDE for deterministic baseline
        target_kl=0.01,
        verbose=1,
        tensorboard_log=None,
        device='auto'
    )

    print("\nModel Configuration:")
    print(f"  Learning Rate: {model.learning_rate}")
    print(f"  Clip Range: {model.clip_range} (conservative for fine-tuning)")
    print(f"  Entropy Coef: {model.ent_coef} (low = follow PN more)")

    # Setup callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=20000,
        save_path=f"{log_dir}/models",
        name_prefix='ppo_hybrid'
    )

    # Train
    print("\n" + "=" * 70)
    print("STARTING TRAINING")
    print("=" * 70)
    print("Agent will learn small improvements to PN guidance...")

    model.learn(
        total_timesteps=total_timesteps,
        callback=checkpoint_callback,
        progress_bar=True
    )

    # Save final model
    final_path = f"{log_dir}/models/ppo_hybrid_final"
    model.save(final_path)
    print(f"\nFinal model saved: {final_path}.zip")

    env.close()

    return model, final_path


def compare_hybrid_vs_pn(model_path, n_episodes=10):
    """Compare hybrid model against pure PN guidance."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

    print("\n" + "=" * 70)
    print("COMPARISON: Hybrid RL vs Pure PN")
    print("=" * 70)

    config = BallistXConfig(
        dt=0.02,
        max_steps=1000,
        max_time=20.0,
        missile_pos=(0.0, 5000.0, 0.0),
        missile_vel=(500.0, 0.0, 0.0),
        missile_max_accel=400.0,
        target_pos=(8000.0, 5000.0, 0.0),
        target_vel=(100.0, 0.0, 0.0),
        proximity_threshold=15.0,
        miss_distance_threshold=20000.0,
    )

    # Test Hybrid
    print("\n--- HYBRID MODEL ---")
    env_hybrid = BallistXEnv(config=config)
    model = PPO.load(model_path, env=env_hybrid)

    hybrid_results = []
    for ep in range(n_episodes):
        obs, info = env_hybrid.reset()
        total_reward = 0

        for step in range(1000):
            # Get PN baseline
            pn_action = info['baseline_accel'] / config.missile_max_accel

            # Get RL correction
            correction, _ = model.predict(obs, deterministic=True)

            # Combine
            final_action = np.clip(pn_action + correction * 0.3, -1.0, 1.0)

            obs, reward, term, trunc, info = env_hybrid.step(final_action)
            total_reward += reward

            if term or trunc:
                break

        hybrid_results.append({
            'reward': total_reward,
            'distance': info['distance'],
            'reason': info['terminated_reason']
        })

        print(f"Episode {ep+1}: Reward={total_reward:7.1f}, "
              f"Dist={info['distance']:6.1f}m, Result={info['terminated_reason']}")

    env_hybrid.close()

    # Test Pure PN
    print("\n--- PURE PN GUIDANCE ---")
    env_pn = BallistXEnv(config=config)

    pn_results = []
    for ep in range(n_episodes):
        obs, info = env_pn.reset()
        total_reward = 0

        for step in range(1000):
            pn_action = info['baseline_accel'] / config.missile_max_accel
            obs, reward, term, trunc, info = env_pn.step(pn_action)
            total_reward += reward

            if term or trunc:
                break

        pn_results.append({
            'reward': total_reward,
            'distance': info['distance'],
            'reason': info['terminated_reason']
        })

        print(f"Episode {ep+1}: Reward={total_reward:7.1f}, "
              f"Dist={info['distance']:6.1f}m, Result={info['terminated_reason']}")

    env_pn.close()

    # Summary
    print("\n" + "=" * 70)
    print("RESULTS SUMMARY")
    print("=" * 70)

    hybrid_avg_reward = np.mean([r['reward'] for r in hybrid_results])
    pn_avg_reward = np.mean([r['reward'] for r in pn_results])
    hybrid_avg_dist = np.mean([r['distance'] for r in hybrid_results])
    pn_avg_dist = np.mean([r['distance'] for r in pn_results])

    print(f"\n{'Metric':<20} {'Hybrid RL':<15} {'Pure PN':<15} {'Winner'}")
    print("-" * 70)
    print(f"{'Avg Reward':<20} {hybrid_avg_reward:>10.1f}     {pn_avg_reward:>10.1f}     "
          f"{'HYBRID' if hybrid_avg_reward > pn_avg_reward else 'PN' if pn_avg_reward > hybrid_avg_reward else 'TIE'}")
    print(f"{'Avg Distance (m)':<20} {hybrid_avg_dist:>10.1f}     {pn_avg_dist:>10.1f}     "
          f"{'HYBRID' if hybrid_avg_dist < pn_avg_dist else 'PN' if pn_avg_dist < hybrid_avg_dist else 'TIE'}")

    improvement = ((hybrid_avg_reward - pn_avg_reward) / abs(pn_avg_reward)) * 100 if pn_avg_reward != 0 else 0
    print(f"\nImprovement over PN: {improvement:+.1f}%")

    if hybrid_avg_reward > pn_avg_reward:
        print("SUCCESS: RL improved upon PN guidance!")
    else:
        print("PN guidance is still optimal - RL couldn't improve it")

    print("=" * 70)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Train Hybrid PN+RL Model')
    parser.add_argument('--timesteps', type=int, default=200000,
                       help='Training timesteps')
    parser.add_argument('--envs', type=int, default=4,
                       help='Parallel environments')
    parser.add_argument('--compare', action='store_true',
                       help='Compare trained model with PN')

    args = parser.parse_args()

    if args.compare:
        compare_hybrid_vs_pn('logs/ppo_hybrid/models/ppo_hybrid_final.zip')
    else:
        model, path = train_hybrid_ppo(
            total_timesteps=args.timesteps,
            n_envs=args.envs
        )
        print("\n" + "=" * 70)
        print("Training complete! To compare with PN, run:")
        print(f"python {__file__} --compare")
        print("=" * 70)
