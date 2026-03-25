"""
PPO Training with CURRICULUM LEARNING

Start with easy scenarios and gradually increase difficulty.
This helps the agent learn basic skills before tackling hard problems.
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
    from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
    from stable_baselines3.common.monitor import Monitor
except ImportError:
    print("Required packages not found. Install with:")
    print("  pip install stable-baselines3 torch")
    sys.exit(1)


def create_easy_env(render_mode=None):
    """Create EASY environment for initial learning."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

    # Very easy scenario: stationary target, close range
    config = BallistXConfig(
        dt=0.02,
        max_steps=500,
        max_time=10.0,
        missile_pos=(0.0, 5000.0, 0.0),
        missile_vel=(300.0, 0.0, 0.0),
        missile_max_accel=400.0,
        target_pos=(3000.0, 5000.0, 0.0),  # Close range
        target_vel=(0.0, 0.0, 0.0),  # Stationary!
        proximity_threshold=50.0,  # Lenient
        miss_distance_threshold=10000.0,
        # Reward shaping: less penalty for crashes
        reward_hit_bonus=500.0,
        reward_miss_penalty=-100.0,
        reward_time_penalty=-0.05,  # Less time pressure
        reward_accel_penalty=-0.001,  # Less energy penalty
    )

    return BallistXEnv(config=config, render_mode=render_mode)


def create_medium_env(render_mode=None):
    """Create MEDIUM environment."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

    # Medium: moving target, medium range
    config = BallistXConfig(
        dt=0.02,
        max_steps=750,
        max_time=15.0,
        missile_pos=(0.0, 5000.0, 0.0),
        missile_vel=(400.0, 0.0, 0.0),
        missile_max_accel=400.0,
        target_pos=(5000.0, 5000.0, 0.0),
        target_vel=(50.0, 0.0, 0.0),  # Slow moving
        proximity_threshold=30.0,
        miss_distance_threshold=15000.0,
        reward_hit_bonus=800.0,
        reward_miss_penalty=-200.0,
        reward_time_penalty=-0.08,
        reward_accel_penalty=-0.005,
    )

    return BallistXEnv(config=config, render_mode=render_mode)


def create_hard_env(render_mode=None):
    """Create HARD environment (original scenario)."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

    # Hard: fast moving target, long range
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
        reward_hit_bonus=1000.0,
        reward_miss_penalty=-500.0,
        reward_time_penalty=-0.1,
        reward_accel_penalty=-0.01,
    )

    return BallistXEnv(config=config, render_mode=render_mode)


def train_curriculum(
    stage='easy',
    total_timesteps=50000,
    n_envs=4,
    load_model=None
):
    """Train with curriculum learning."""

    print("=" * 70)
    print(f"CURRICULUM TRAINING - STAGE: {stage.upper()}")
    print("=" * 70)

    # Select environment based on stage
    if stage == 'easy':
        env_fn = create_easy_env
        log_dir = 'logs/ppo_easy'
    elif stage == 'medium':
        env_fn = create_medium_env
        log_dir = 'logs/ppo_medium'
    else:  # hard
        env_fn = create_hard_env
        log_dir = 'logs/ppo_hard'

    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(f"{log_dir}/models", exist_ok=True)

    print(f"Environment: {stage}")
    print(f"Total Timesteps: {total_timesteps:,}")
    print(f"Log Directory: {log_dir}")
    print("=" * 70)

    # Create vectorized environments
    def make_monitored_env():
        env = env_fn()
        return Monitor(env, log_dir, allow_early_resets=True)

    env = make_vec_env(make_monitored_env, n_envs=n_envs)

    # Create or load model
    if load_model:
        print(f"\nLoading existing model: {load_model}")
        model = PPO.load(load_model, env=env, tensorboard_log=None)
    else:
        print("\nCreating new PPO model...")
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=1e-4,  # Lower learning rate for stability
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            clip_range_vf=None,
            normalize_advantage=True,
            ent_coef=0.01,  # Some exploration
            vf_coef=0.5,
            max_grad_norm=0.5,
            use_sde=True,
            sde_sample_freq=4,
            target_kl=0.01,
            verbose=1,
            tensorboard_log=None,
            device='auto'
        )

    print("\nModel Configuration:")
    print(f"  Policy: MlpPolicy")
    print(f"  Learning Rate: {model.learning_rate}")
    print(f"  Batch Size: {model.batch_size}")
    print(f"  Entropy Coef: {model.ent_coef}")

    # Setup callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path=f"{log_dir}/models",
        name_prefix=f'ppo_{stage}'
    )

    # Train
    print("\n" + "=" * 70)
    print("STARTING TRAINING")
    print("=" * 70)

    model.learn(
        total_timesteps=total_timesteps,
        callback=checkpoint_callback,
        reset_num_timesteps=(load_model is None),
        progress_bar=True
    )

    # Save final model
    final_path = f"{log_dir}/models/ppo_{stage}_final"
    model.save(final_path)
    print(f"\nFinal model saved: {final_path}.zip")

    env.close()

    return model, final_path


def main():
    parser = argparse.ArgumentParser(description='Curriculum PPO Training')
    parser.add_argument('--stage', type=str, default='easy',
                       choices=['easy', 'medium', 'hard'],
                       help='Training stage')
    parser.add_argument('--timesteps', type=int, default=50000,
                       help='Training timesteps')
    parser.add_argument('--envs', type=int, default=4,
                       help='Parallel environments')
    parser.add_argument('--load', type=str, default=None,
                       help='Load existing model')
    parser.add_argument('--eval', action='store_true',
                       help='Run evaluation after training')

    args = parser.parse_args()

    # Train
    model, model_path = train_curriculum(
        stage=args.stage,
        total_timesteps=args.timesteps,
        n_envs=args.envs,
        load_model=args.load
    )

    # Evaluate if requested
    if args.eval:
        print("\n" + "=" * 70)
        print("EVALUATION")
        print("=" * 70)

        env = (create_easy_env() if args.stage == 'easy' else
                create_medium_env() if args.stage == 'medium' else
                create_hard_env())

        obs, info = env.reset()
        episode_rewards = []

        for episode in range(10):
            obs, info = env.reset()
            episode_reward = 0
            done = False
            steps = 0

            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, done, truncated, info = env.step(action)
                episode_reward += reward
                steps += 1
                if truncated:
                    break

            episode_rewards.append(episode_reward)
            print(f"Episode {episode+1}: Reward={episode_reward:.1f}, "
                  f"Steps={steps}, Dist={info['distance']:.1f}m, "
                  f"Result={info['terminated_reason']}")

        print(f"\nAverage Reward: {np.mean(episode_rewards):.1f} +/- {np.std(episode_rewards):.1f}")
        env.close()


if __name__ == '__main__':
    main()
