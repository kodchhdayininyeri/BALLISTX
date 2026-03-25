"""
PPO Training Script for BALLISTX Missile Guidance

Train a Proximal Policy Optimization agent for continuous action control.
PPO is well-suited for continuous control tasks like missile guidance.

Usage:
    python train_ppo.py --timesteps 200000
    python train_ppo.py --load models/ppo_ballistx.zip --eval --render
"""

import sys
from pathlib import Path

# Add ballistx_gym package to path
gym_path = Path(__file__).parent.parent
sys.path.insert(0, str(gym_path))

import numpy as np
import argparse
import os

try:
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.results_plotter import load_results, ts2xy
    import matplotlib.pyplot as plt
except ImportError:
    print("Required packages not found. Install with:")
    print("  pip install stable-baselines3 torch matplotlib")
    sys.exit(1)


def create_env(scenario='tail_chase', difficulty=0.5, render_mode=None):
    """Create BALLISTX environment with specified configuration."""
    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

    # Achievable tail-chase scenario: missile can intercept in 20s
    # Closing speed = 500 - 100 = 400 m/s, covering 8000m in exactly 20s
    config = BallistXConfig(
        dt=0.02,
        max_steps=1000,  # 20 seconds
        max_time=20.0,
        missile_max_accel=400.0,
        proximity_threshold=15.0,
        miss_distance_threshold=20000.0,
        missile_pos=(0.0, 5000.0, 0.0),
        missile_vel=(500.0, 0.0, 0.0),  # Fast missile
        target_pos=(8000.0, 5000.0, 0.0),
        target_vel=(100.0, 0.0, 0.0),  # Slow target
    )

    # Apply difficulty
    speed_multiplier = 1.0 + difficulty * 0.5
    base_vel = np.array(config.target_vel)
    config.target_vel = tuple(base_vel * speed_multiplier)
    config.proximity_threshold = 20.0 * (1.0 - difficulty * 0.3)

    return BallistXEnv(config=config, render_mode=render_mode)


def train_ppo(
    total_timesteps=200000,
    scenario='tail_chase',
    difficulty=0.5,
    n_envs=4,
    log_dir='logs/ppo',
    checkpoint_freq=20000,
    eval_freq=10000,
    load_model=None
):
    """
    Train PPO agent on BALLISTX environment.

    PPO is particularly good for continuous control tasks and is more
    sample-efficient than DQN for continuous action spaces.

    Args:
        total_timesteps: Total training timesteps
        scenario: Engagement scenario
        difficulty: Difficulty level [0.0, 1.0]
        n_envs: Number of parallel environments
        log_dir: Logging directory
        checkpoint_freq: Save checkpoint every N steps
        eval_freq: Evaluation frequency
        load_model: Path to existing model to continue training
    """
    # Create log directory
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(f"{log_dir}/models", exist_ok=True)

    print("=" * 70)
    print("PPO TRAINING - BALLISTX MISSILE GUIDANCE")
    print("=" * 70)
    print(f"Scenario: {scenario}")
    print(f"Difficulty: {difficulty}")
    print(f"Total Timesteps: {total_timesteps:,}")
    print(f"Parallel Environments: {n_envs}")
    print(f"Log Directory: {log_dir}")
    print("=" * 70)

    # Create vectorized environments
    print("\nCreating environments...")
    env_fn = lambda: create_env(scenario, difficulty)
    env = make_vec_env(env_fn, n_envs=n_envs)

    # Wrap each individual environment with Monitor
    def make_monitored_env():
        env = create_env(scenario, difficulty)
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
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            clip_range_vf=None,
            normalize_advantage=True,
            ent_coef=0.0,
            vf_coef=0.5,
            max_grad_norm=0.5,
            use_sde=True,  # State-dependent exploration for better continuous control
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
    print(f"  Gamma: {model.gamma}")
    print(f"  GAE Lambda: {model.gae_lambda}")
    print(f"  Clip Range: {model.clip_range}")
    print(f"  Use SDE: {model.use_sde}")
    print(f"  Entropy Coef: {model.ent_coef}")

    # Setup callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_freq,
        save_path=f"{log_dir}/models",
        name_prefix='ppo_ballistx'
    )

    # Evaluation environment
    def make_eval_env():
        return Monitor(create_env(scenario, difficulty), f"{log_dir}/eval", allow_early_resets=True)

    eval_env = make_vec_env(make_eval_env, n_envs=1)

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=f"{log_dir}/models/best",
        log_path=f"{log_dir}/eval",
        eval_freq=eval_freq,
        deterministic=True,
        render=False,
        n_eval_episodes=10,
        verbose=1
    )

    # Train
    print("\n" + "=" * 70)
    print("STARTING TRAINING")
    print("=" * 70)
    print(f"Press Ctrl+C to stop and save checkpoint\n")

    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=[checkpoint_callback, eval_callback],
            reset_num_timesteps=(load_model is None),
            progress_bar=True
        )
    except KeyboardInterrupt:
        print("\n\nTraining interrupted by user. Saving checkpoint...")

    # Save final model
    final_path = f"{log_dir}/models/ppo_ballistx_final"
    model.save(final_path)
    print(f"\nFinal model saved: {final_path}.zip")

    # Close environments
    env.close()
    eval_env.close()

    return model


def evaluate_model(model_path, scenario='tail_chase', difficulty=0.5, n_episodes=20, render=False):
    """Evaluate a trained PPO model."""
    print("=" * 70)
    print("PPO MODEL EVALUATION")
    print("=" * 70)

    # Create environment
    env = create_env(scenario, difficulty, render_mode='human' if render else None)

    # Load model
    print(f"\nLoading model: {model_path}")
    model = PPO.load(model_path, env=env)

    # Evaluate
    results = []

    for episode in range(1, n_episodes + 1):
        obs, info = env.reset()
        episode_reward = 0.0
        episode_steps = 0
        done = False
        truncated = False

        while not (done or truncated):
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
            episode_reward += reward
            episode_steps += 1

        results.append({
            'episode': episode,
            'reward': episode_reward,
            'steps': episode_steps,
            'final_distance': info['distance'],
            'reason': info['terminated_reason'],
            'hit': info.get('is_hit', False)
        })

        print(f"Episode {episode:2d}: Reward={episode_reward:7.1f}, "
              f"Steps={episode_steps:3d}, Distance={info['distance']:6.1f}m, "
              f"Result={info['terminated_reason'].upper()}")

    env.close()

    # Print statistics
    print("\n" + "=" * 70)
    print("EVALUATION RESULTS")
    print("=" * 70)

    hits = sum(1 for r in results if r['hit'])
    avg_reward = np.mean([r['reward'] for r in results])
    avg_steps = np.mean([r['steps'] for r in results])
    avg_distance = np.mean([r['final_distance'] for r in results])

    print(f"\nEpisodes: {n_episodes}")
    print(f"Hits: {hits}/{n_episodes} ({100*hits/n_episodes:.1f}%)")
    print(f"Avg Reward: {avg_reward:.1f}")
    print(f"Avg Steps: {avg_steps:.1f}")
    print(f"Avg Final Distance: {avg_distance:.1f}m")
    print("=" * 70)

    return results


def compare_models(dqn_path, ppo_path, scenario='tail_chase', difficulty=0.5, n_episodes=20):
    """Compare DQN vs PPO models."""
    print("=" * 70)
    print("MODEL COMPARISON: DQN vs PPO")
    print("=" * 70)

    from stable_baselines3 import DQN, PPO

    # Create environment
    env = create_env(scenario, difficulty)

    # Load models
    print(f"\nLoading DQN: {dqn_path}")
    dqn_model = DQN.load(dqn_path, env=env)

    print(f"Loading PPO: {ppo_path}")
    ppo_model = PPO.load(ppo_path, env=env)

    # Evaluate DQN
    print("\n--- Evaluating DQN ---")
    dqn_results = []
    for episode in range(n_episodes):
        obs, info = env.reset()
        total_reward = 0
        done, truncated = False, False

        while not (done or truncated):
            action, _ = dqn_model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
            total_reward += reward

        dqn_results.append({'reward': total_reward, 'distance': info['distance'], 'hit': info.get('is_hit', False)})

    dqn_hits = sum(1 for r in dqn_results if r['hit'])
    dqn_avg_reward = np.mean([r['reward'] for r in dqn_results])
    dqn_avg_dist = np.mean([r['distance'] for r in dqn_results])

    print(f"DQN Results ({n_episodes} episodes):")
    print(f"  Hits: {dqn_hits}/{n_episodes} ({100*dqn_hits/n_episodes:.1f}%)")
    print(f"  Avg Reward: {dqn_avg_reward:.1f}")
    print(f"  Avg Final Distance: {dqn_avg_dist:.1f}m")

    # Evaluate PPO
    print("\n--- Evaluating PPO ---")
    ppo_results = []
    for episode in range(n_episodes):
        obs, info = env.reset()
        total_reward = 0
        done, truncated = False, False

        while not (done or truncated):
            action, _ = ppo_model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
            total_reward += reward

        ppo_results.append({'reward': total_reward, 'distance': info['distance'], 'hit': info.get('is_hit', False)})

    ppo_hits = sum(1 for r in ppo_results if r['hit'])
    ppo_avg_reward = np.mean([r['reward'] for r in ppo_results])
    ppo_avg_dist = np.mean([r['distance'] for r in ppo_results])

    print(f"PPO Results ({n_episodes} episodes):")
    print(f"  Hits: {ppo_hits}/{n_episodes} ({100*ppo_hits/n_episodes:.1f}%)")
    print(f"  Avg Reward: {ppo_avg_reward:.1f}")
    print(f"  Avg Final Distance: {ppo_avg_dist:.1f}m")

    env.close()

    print("\n" + "=" * 70)
    print("COMPARISON SUMMARY")
    print("=" * 70)
    print(f"{'Metric':<20} {'DQN':<15} {'PPO':<15} {'Winner'}")
    print("-" * 70)
    print(f"{'Hit Rate':<20} {100*dqn_hits/n_episodes:>6.1f}%      {100*ppo_hits/n_episodes:>6.1f}%      {'DQN' if dqn_hits > ppo_hits else 'PPO' if ppo_hits > dqn_hits else 'TIE'}")
    print(f"{'Avg Reward':<20} {dqn_avg_reward:>10.1f}     {ppo_avg_reward:>10.1f}     {'PPO' if ppo_avg_reward > dqn_avg_reward else 'DQN' if dqn_avg_reward > ppo_avg_reward else 'TIE'}")
    print(f"{'Avg Distance':<20} {dqn_avg_dist:>10.1f}m    {ppo_avg_dist:>10.1f}m    {'PPO' if dpo_avg_dist < dqn_avg_dist else 'DQN' if dqn_avg_dist < ppo_avg_dist else 'TIE'}")
    print("=" * 70)

    return {'dqn': dqn_results, 'ppo': ppo_results}


def main():
    parser = argparse.ArgumentParser(description='Train PPO on BALLISTX environment')

    # Training arguments
    parser.add_argument('--timesteps', type=int, default=200000,
                       help='Total training timesteps')
    parser.add_argument('--scenario', type=str, default='tail_chase',
                       choices=['head_on', 'tail_chase', 'crossing', 'maneuvering', 'stationary'],
                       help='Engagement scenario')
    parser.add_argument('--difficulty', type=float, default=0.5,
                       help='Difficulty level [0.0, 1.0]')
    parser.add_argument('--envs', type=int, default=4,
                       help='Number of parallel environments')
    parser.add_argument('--log-dir', type=str, default='logs/ppo',
                       help='Logging directory')

    # Model loading
    parser.add_argument('--load', type=str, default=None,
                       help='Load existing model and continue training')

    # Evaluation mode
    parser.add_argument('--eval', action='store_true',
                       help='Run evaluation instead of training')
    parser.add_argument('--eval-model', type=str, default=None,
                       help='Model path for evaluation')
    parser.add_argument('--episodes', type=int, default=20,
                       help='Number of evaluation episodes')
    parser.add_argument('--render', action='store_true',
                       help='Render during evaluation')

    # Model comparison
    parser.add_argument('--compare', action='store_true',
                       help='Compare DQN vs PPO models')
    parser.add_argument('--dqn-model', type=str,
                       help='DQN model path for comparison')
    parser.add_argument('--ppo-model', type=str,
                       help='PPO model path for comparison')

    args = parser.parse_args()

    if args.compare:
        if not args.dqn_model or not args.ppo_model:
            print("Both --dqn-model and --ppo-model required for comparison")
            return
        compare_models(args.dqn_model, args.ppo_model, args.scenario, args.difficulty, args.episodes)
    elif args.eval:
        if args.eval_model is None:
            import glob
            checkpoints = glob.glob(f"{args.log_dir}/models/ppo_ballistx_*.zip")
            if checkpoints:
                args.eval_model = max(checkpoints, key=os.path.getctime)
                print(f"Using latest checkpoint: {args.eval_model}")
            else:
                print("No checkpoint found. Please specify --eval-model")
                return

        evaluate_model(
            model_path=args.eval_model,
            scenario=args.scenario,
            difficulty=args.difficulty,
            n_episodes=args.episodes,
            render=args.render
        )
    else:
        train_ppo(
            total_timesteps=args.timesteps,
            scenario=args.scenario,
            difficulty=args.difficulty,
            n_envs=args.envs,
            log_dir=args.log_dir,
            load_model=args.load
        )


if __name__ == '__main__':
    main()
