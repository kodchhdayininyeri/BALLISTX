"""
DQN Training Script for BALLISTX Missile Guidance

Train a Deep Q-Network agent to learn missile guidance using
the BALLISTX Gymnasium environment.

Usage:
    python train_dqn.py --timesteps 100000 --env tail_chase
    python train_dqn.py --load models/dqn_ballistx.zip --eval
"""

import sys
from pathlib import Path

# Add ballistx_gym package to path
gym_path = Path(__file__).parent.parent
sys.path.insert(0, str(gym_path))

import numpy as np
import argparse
from datetime import datetime
import os

try:
    import torch
    from stable_baselines3 import DQN
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
    from ballistx_gym.discrete_ballistx_env import DiscreteBallistXEnv, BallistXConfig

    # Achievable tail-chase scenario: missile can intercept in 20s
    # Closing speed = 500 - 100 = 400 m/s, covering 8000m in exactly 20s
    config = BallistXConfig(
        dt=0.02,
        max_steps=1000,  # 20 seconds at dt=0.02
        max_time=20.0,
        missile_max_accel=400.0,
        proximity_threshold=15.0,
        miss_distance_threshold=20000.0,
        missile_pos=(0.0, 5000.0, 0.0),
        missile_vel=(500.0, 0.0, 0.0),  # Fast missile
        target_pos=(8000.0, 5000.0, 0.0),
        target_vel=(100.0, 0.0, 0.0),  # Slow target
    )

    # Apply difficulty - affects target speed and proximity threshold
    speed_multiplier = 1.0 + difficulty * 0.5  # 1.0 to 1.5x
    base_vel = np.array(config.target_vel)
    config.target_vel = tuple(base_vel * speed_multiplier)
    config.proximity_threshold = 20.0 * (1.0 - difficulty * 0.3)  # 20m to 14m

    return DiscreteBallistXEnv(config=config, render_mode=render_mode)


def train_dqn(
    total_timesteps=100000,
    scenario='tail_chase',
    difficulty=0.5,
    n_envs=4,
    log_dir='logs/dqn',
    checkpoint_freq=10000,
    eval_freq=5000,
    load_model=None
):
    """
    Train DQN agent on BALLISTX environment.

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
    print("DQN TRAINING - BALLISTX MISSILE GUIDANCE")
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

    # Wrap each individual environment with Monitor before vec_env
    # We'll create a new function that wraps with Monitor
    def make_monitored_env():
        env = create_env(scenario, difficulty)
        return Monitor(env, log_dir, allow_early_resets=True)

    env = make_vec_env(make_monitored_env, n_envs=n_envs)

    # Create or load model
    if load_model:
        print(f"\nLoading existing model: {load_model}")
        model = DQN.load(load_model, env=env, tensorboard_log=None)
    else:
        print("\nCreating new DQN model...")
        model = DQN(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            buffer_size=100000,
            learning_starts=1000,
            batch_size=64,
            gamma=0.99,
            exploration_fraction=0.1,
            exploration_initial_eps=1.0,
            exploration_final_eps=0.05,
            train_freq=4,
            gradient_steps=1,
            target_update_interval=1000,
            verbose=1,
            tensorboard_log=None,  # Disable tensorboard for now
            device='auto'  # Use GPU if available
        )

    print("\nModel Configuration:")
    print(f"  Policy: MlpPolicy")
    print(f"  Learning Rate: {model.learning_rate}")
    print(f"  Buffer Size: {model.buffer_size}")
    print(f"  Batch Size: {model.batch_size}")
    print(f"  Gamma: {model.gamma}")
    print(f"  Exploration: {model.exploration_fraction:.1%}")

    # Setup callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_freq,
        save_path=f"{log_dir}/models",
        name_prefix='dqn_ballistx'
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
    final_path = f"{log_dir}/models/dqn_ballistx_final"
    model.save(final_path)
    print(f"\nFinal model saved: {final_path}.zip")

    # Close environments
    env.close()
    eval_env.close()

    return model


def evaluate_model(model_path, scenario='tail_chase', difficulty=0.5, n_episodes=20, render=False):
    """
    Evaluate a trained model.

    Args:
        model_path: Path to trained model
        scenario: Engagement scenario
        difficulty: Difficulty level
        n_episodes: Number of evaluation episodes
        render: Whether to render episodes
    """
    print("=" * 70)
    print("MODEL EVALUATION")
    print("=" * 70)

    # Create environment
    env = create_env(scenario, difficulty, render_mode='human' if render else None)

    # Load model
    print(f"\nLoading model: {model_path}")
    model = DQN.load(model_path, env=env)

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
            # Ensure action is an integer for discrete environment
            action = int(action.item()) if hasattr(action, 'item') else int(action)
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


def plot_training_logs(log_dir='logs/dqn'):
    """Plot training curves from TensorBoard logs."""
    import matplotlib.pyplot as plt

    print(f"\nLoading training logs from: {log_dir}")

    # Load monitor.csv files
    x, y = ts2xy(load_results(log_dir, convex=False), 'timesteps')

    # Filter out eval data (from eval/ subdirectory)
    train_x = []
    train_y = []
    for xi, yi in zip(x, y):
        if 'eval' not in str(xi):
            train_x.append(xi)
            train_y.append(yi)

    # Create figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # Episode rewards
    ax1.plot(train_x, train_y, alpha=0.3)
    # Smooth with rolling mean
    if len(train_y) > 100:
        window = 100
        smoothed = np.convolve(train_y, np.ones(window)/window, mode='valid')
        smoothed_x = train_x[window-1:]
        ax1.plot(smoothed_x, smoothed, linewidth=2, label=f'Rolling Mean (window={window})')

    ax1.set_xlabel('Timesteps')
    ax1.set_ylabel('Episode Reward')
    ax1.set_title('Training Progress')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Histogram of final rewards
    ax2.hist(train_y[-1000:] if len(train_y) > 1000 else train_y, bins=50, alpha=0.7)
    ax2.set_xlabel('Episode Reward')
    ax2.set_ylabel('Count')
    ax2.set_title('Reward Distribution (Last 1000 episodes)')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    plot_path = f"{log_dir}/training_plot.png"
    plt.savefig(plot_path, dpi=100)
    print(f"Training plot saved: {plot_path}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Train DQN on BALLISTX environment')

    # Training arguments
    parser.add_argument('--timesteps', type=int, default=100000,
                       help='Total training timesteps')
    parser.add_argument('--scenario', type=str, default='tail_chase',
                       choices=['head_on', 'tail_chase', 'crossing', 'maneuvering', 'stationary'],
                       help='Engagement scenario')
    parser.add_argument('--difficulty', type=float, default=0.5,
                       help='Difficulty level [0.0, 1.0]')
    parser.add_argument('--envs', type=int, default=4,
                       help='Number of parallel environments')
    parser.add_argument('--log-dir', type=str, default='logs/dqn',
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

    # Plotting
    parser.add_argument('--plot', action='store_true',
                       help='Plot training curves')

    args = parser.parse_args()

    if args.plot:
        plot_training_logs(args.log_dir)
    elif args.eval:
        if args.eval_model is None:
            # Use latest checkpoint
            import glob
            checkpoints = glob.glob(f"{args.log_dir}/models/dqn_ballistx_*.zip")
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
        train_dqn(
            total_timesteps=args.timesteps,
            scenario=args.scenario,
            difficulty=args.difficulty,
            n_envs=args.envs,
            log_dir=args.log_dir,
            load_model=args.load
        )


if __name__ == '__main__':
    main()
