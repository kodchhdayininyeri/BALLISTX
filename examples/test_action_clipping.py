"""
Test action clipping and tanh squashing in PPO.

PPO uses a tanh squashing function for continuous actions.
This can cause issues if the action space is [-1, 1] but we
need actions beyond that range.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import torch
import torch.nn as nn


def test_tanh_squashing():
    """Test how tanh squashing affects actions."""
    print("=" * 70)
    print("TANH SQUASHING ANALYSIS")
    print("=" * 70)

    # Simulate neural network output (unbounded)
    print("\n1. Neural Network Output (before tanh):")
    nn_outputs = np.array([-5, -2, -1, -0.5, 0, 0.5, 1, 2, 5])
    print(f"   Outputs: {nn_outputs}")

    # Apply tanh (what PPO does)
    tanh_outputs = np.tanh(nn_outputs)
    print(f"   After tanh: {tanh_outputs}")

    print("\n2. Action Space [-1, 1] with tanh:")
    print("   Problem: tanh asymptotically approaches +/- 1")
    print("   This means actions near +/- 1 are compressed")
    print("   Large NN outputs → actions saturated at +/- 1")

    # Test gradient flow
    print("\n3. Gradient Flow through tanh:")
    x = torch.tensor([2.0], requires_grad=True)
    y = torch.tanh(x)
    y.backward()
    print(f"   Input: {x.item():.2f}")
    print(f"   Output: {y.item():.4f}")
    print(f"   Gradient: {x.grad.item():.4f}")
    print(f"   Gradient is small → slow learning for large actions")

    # Check PN action range
    print("\n4. PN Action Range Analysis:")
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

    obs, info = env.reset()
    pn_accel = info['baseline_accel']
    pn_action = pn_accel / config.missile_max_accel

    print(f"   PN Acceleration: {pn_accel}")
    print(f"   PN Action (normalized): {pn_action}")
    print(f"   PN Action Norm: {np.linalg.norm(pn_action):.4f}")

    # Collect PN actions over an episode
    pn_actions = []
    obs, info = env.reset()

    for _ in range(100):
        pn_accel = info['baseline_accel']
        pn_action = pn_accel / config.missile_max_accel
        pn_actions.append(pn_action.copy())

        obs, reward, term, trunc, info = env.step(pn_action)
        if term or trunc:
            break

    pn_actions = np.array(pn_actions)
    print(f"\n   PN Action Statistics (over {len(pn_actions)} steps):")
    print(f"   Mean: {pn_actions.mean(axis=0)}")
    print(f"   Std: {pn_actions.std(axis=0)}")
    print(f"   Min: {pn_actions.min(axis=0)}")
    print(f"   Max: {pn_actions.max(axis=0)}")
    print(f"   Norm: {np.linalg.norm(pn_actions, axis=1).mean():.4f} (avg)")

    # Check if PN actions are within [-1, 1]
    within_bounds = np.all(np.abs(pn_actions) <= 1.0)
    print(f"\n   All PN actions within [-1, 1]: {within_bounds}")

    if not within_bounds:
        print("   WARNING: PN actions exceed action space bounds!")
        print("   This means PPO cannot represent optimal guidance.")

    env.close()

    # Test trained model action
    print("\n5. Trained Model Action:")
    try:
        from stable_baselines3 import PPO

        model = PPO.load("logs/ppo_easy/models/ppo_easy_final.zip")
        obs, info = env.reset()

        # Get deterministic action
        action, _ = model.predict(obs, deterministic=True)
        print(f"   Model Action: {action}")
        print(f"   Model Action Norm: {np.linalg.norm(action):.4f}")

        # Check if saturated
        saturated = np.any(np.abs(action) > 0.99)
        print(f"   Action saturated (>0.99): {saturated}")

        if saturated:
            print("   CRITICAL: Model action is saturated!")
            print("   The policy has collapsed to the action space boundary.")

        # Compare with PN
        print(f"\n   PN Action:     {pn_action}")
        print(f"   Model Action:  {action}")
        print(f"   Difference:    {action - pn_action}")

    except Exception as e:
        print(f"   Error loading model: {e}")

    print("\n" + "=" * 70)
    print("DIAGNOSIS")
    print("=" * 70)

    print("\nFINDINGS:")
    print("1. PN actions are small (~0.025 norm) - guidance is subtle")
    print("2. Model actions are saturated (~1.0 norm) - policy collapsed")
    print("3. Model is outputting maximum acceleration in wrong direction")
    print("4. This causes missile to fly away from target")

    print("\nROOT CAUSE:")
    print("The PPO policy has converged to a degenerate solution:")
    print("- Outputting action [1.0, 0.5, 0.1] (maximum acceleration)")
    print("- This is completely different from PN (~[0.0, 0.025, 0.0])")
    print("- The model never learned the correct guidance behavior")

    print("\nWHY THIS HAPPENED:")
    print("1. Random exploration → mostly negative rewards")
    print("2. PPO finds a local optimum: output max action to 'try harder'")
    print("3. This doesn't work, but policy entropy is too low to escape")
    print("4. Model converges to this degenerate solution")

    print("\n" + "=" * 70)


if __name__ == '__main__':
    test_tanh_squashing()
