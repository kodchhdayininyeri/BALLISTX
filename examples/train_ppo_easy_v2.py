"""
PPO Training with BETTER HYPERPARAMETERS

Fixed version with:
- Higher exploration (entropy coefficient)
- Lower learning rate (more stable)
- More timesteps (longer training)
- Better initialization
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import argparse
import os

try:
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.monitor import Monitor
except ImportError:
    print("Required packages not found. Install with:")
    print("  pip install stable-baselines3 torch")
    sys.exit(1)


def create_easy_env(render_mode=None):
    """Create EASY environment."""
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

    return BallistXEnv(config=config, render_mode=render_mode)


def train_ppo_better(total_timesteps=200000, n_envs=4):
    """Train PPO with better hyperparameters."""

    log_dir = 'logs/ppo_easy_v2'
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(f"{log_dir}/models", exist_ok=True)

    print("=" * 70)
    print("PPO TRAINING V2 - BETTER HYPERPARAMETERS")
    print("=" * 70)
    print(f"Total Timesteps: {total_timesteps:,}")
    print(f"Log Directory: {log_dir}")
    print("=" * 70)

    # Create vectorized environments
    def make_monitored_env():
        env = create_easy_env()
        return Monitor(env, log_dir, allow_early_resets=True)

    env = make_vec_env(make_monitored_env, n_envs=n_envs)

    # BETTER HYPERPARAMETERS
    print("\nCreating PPO model with BETTER hyperparameters...")
    model = PPO(
        "MlpPolicy",
        env,
        # === CRITICAL CHANGES ===
        learning_rate=3e-4,        # Standard, not too low
        n_steps=2048,               # Standard
        batch_size=64,              # Standard
        n_epochs=10,                # Standard

        # === EXPLORATION (KEY!) ===
        ent_coef=0.05,              # INCREASED from 0.01 - MORE EXPLORATION!
        use_sde=True,               # State-dependent exploration
        sde_sample_freq=4,          # Update noise every 4 steps

        # === STABILITY ===
        gamma=0.99,                 # Discount factor
        gae_lambda=0.95,            # GAE
        clip_range=0.2,             # PPO clip
        max_grad_norm=0.5,          # Gradient clipping

        # === VALUE FUNCTION ===
        vf_coef=0.5,                # Value loss weight
        clip_range_vf=None,         # Don't clip value function

        # === OTHER ===
        normalize_advantage=True,   # Normalize advantages
        target_kl=0.01,             # KL divergence target
        verbose=1,
        tensorboard_log=None,
        device='auto'
    )

    print("\nModel Configuration:")
    print(f"  Learning Rate: {model.learning_rate}")
    print(f"  Entropy Coef: {model.ent_coef} [INCREASED FOR EXPLORATION]")
    print(f"  Use SDE: {model.use_sde}")
    print(f"  Batch Size: {model.batch_size}")

    # Callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=20000,
        save_path=f"{log_dir}/models",
        name_prefix='ppo_easy_v2'
    )

    # Train
    print("\n" + "=" * 70)
    print("STARTING TRAINING")
    print("=" * 70)
    print("Watch for 'ep_rew_mean' - it should INCREASE during training!")
    print("=" * 70)

    model.learn(
        total_timesteps=total_timesteps,
        callback=checkpoint_callback,
        progress_bar=True
    )

    # Save
    final_path = f"{log_dir}/models/ppo_easy_v2_final"
    model.save(final_path)
    print(f"\nFinal model saved: {final_path}.zip")

    env.close()

    return model


def evaluate_model(model_path, n_episodes=10):
    """Evaluate trained model."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

    print("\n" + "=" * 70)
    print("EVALUATION")
    print("=" * 70)

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
    model = PPO.load(model_path, env=env)

    results = []
    for episode in range(n_episodes):
        obs, info = env.reset()
        episode_reward = 0
        steps = 0

        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, term, trunc, info = env.step(action)
            episode_reward += reward
            steps += 1

            if term or trunc:
                results.append({
                    'episode': episode + 1,
                    'reward': episode_reward,
                    'steps': steps,
                    'distance': info['distance'],
                    'reason': info['terminated_reason']
                })
                break

        print(f"Episode {episode+1}: Reward={episode_reward:.1f}, "
              f"Steps={steps}, Dist={info['distance']:.1f}m, "
              f"Result={info['terminated_reason']}")

    env.close()

    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)
    print(f"Avg Reward: {np.mean([r['reward'] for r in results]):.1f}")
    print(f"Hit Rate: {sum(1 for r in results if r['reason'] == 'hit')}/{n_episodes}")
    print(f"Avg Distance: {np.mean([r['distance'] for r in results]):.1f}m")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timesteps', type=int, default=200000)
    parser.add_argument('--envs', type=int, default=4)
    parser.add_argument('--eval-only', action='store_true')
    parser.add_argument('--eval-model', type=str, default=None)

    args = parser.parse_args()

    if args.eval_only:
        model_path = args.eval_model or 'logs/ppo_easy_v2/models/ppo_easy_v2_final.zip'
        evaluate_model(model_path)
    else:
        model = train_ppo_better(args.timesteps, args.envs)
        # Auto-evaluate after training
        evaluate_model('logs/ppo_easy_v2/models/ppo_easy_v2_final.zip')


if __name__ == '__main__':
    main()
