"""
BALLISTX Gymnasium Environment Test

Test script for the BALLISTX RL environment.
Run this to verify the environment is working correctly.
"""

import sys
from pathlib import Path

# Add gymnasium package to path
gym_path = Path(__file__).parent.parent
sys.path.insert(0, str(gym_path))

import numpy as np


def test_import():
    """Test that environment can be imported."""
    print("Testing import...")
    try:
        import ballistx_gym as gym
        from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig
        print("  [OK] Import successful")
        return True
    except Exception as e:
        print(f"  [FAIL] Import failed: {e}")
        return False


def test_environment_creation():
    """Test environment creation."""
    print("\nTesting environment creation...")
    try:
        from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

        # Test with default config
        env = BallistXEnv()
        print(f"  [OK] Action space: {env.action_space}")
        print(f"  [OK] Observation space: {env.observation_space}")
        print(f"  [OK] Observation shape: {env.observation_space.shape}")

        env.close()
        return True
    except Exception as e:
        print(f"  [FAIL] Environment creation failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_reset():
    """Test environment reset."""
    print("\nTesting reset()...")
    try:
        from ballistx_gym.ballistx_env import BallistXEnv

        env = BallistXEnv()

        obs, info = env.reset()

        print(f"  [OK] Observation shape: {obs.shape}")
        print(f"  [OK] Observation dtype: {obs.dtype}")
        print(f"  [OK] Info keys: {list(info.keys())}")
        print(f"  [OK] Initial distance: {info['distance']:.1f}m")

        env.close()
        return True
    except Exception as e:
        print(f"  [FAIL] Reset failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_step():
    """Test environment step."""
    print("\nTesting step()...")
    try:
        from ballistx_gym.ballistx_env import BallistXEnv

        env = BallistXEnv()
        obs, info = env.reset()

        # Take a random action
        action = env.action_space.sample()
        print(f"  [OK] Action sampled: {action}")

        # Step environment
        next_obs, reward, terminated, truncated, info = env.step(action)

        print(f"  [OK] Next observation shape: {next_obs.shape}")
        print(f"  [OK] Reward: {reward:.2f}")
        print(f"  [OK] Terminated: {terminated}")
        print(f"  [OK] Truncated: {truncated}")
        print(f"  [OK] New distance: {info['distance']:.1f}m")

        env.close()
        return True
    except Exception as e:
        print(f"  [FAIL] Step failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_random_agent():
    """Test with random agent for a few steps."""
    print("\nTesting random agent (20 steps)...")
    try:
        from ballistx_gym.ballistx_env import BallistXEnv

        env = BallistXEnv()
        obs, info = env.reset()

        total_reward = 0.0
        distances = []

        for step in range(20):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)

            total_reward += reward
            distances.append(info['distance'])

            if terminated or truncated:
                print(f"  Episode ended at step {step}: {info['terminated_reason']}")
                break

        print(f"  [OK] Total reward: {total_reward:.2f}")
        print(f"  [OK] Final distance: {distances[-1]:.1f}m")
        print(f"  [OK] Distance reduction: {distances[0] - distances[-1]:.1f}m")

        env.close()
        return True
    except Exception as e:
        print(f"  [FAIL] Random agent test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_custom_config():
    """Test environment with custom configuration."""
    print("\nTesting custom configuration...")
    try:
        from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig

        config = BallistXConfig(
            dt=0.02,
            max_steps=500,
            missile_pos=(0.0, 3000.0, 0.0),
            missile_vel=(400.0, 0.0, 0.0),
            target_pos=(5000.0, 3000.0, 0.0),
            target_vel=(100.0, 0.0, 0.0),
            missile_max_accel=300.0,
            guidance_gain=4.0,
            proximity_threshold=5.0,
        )

        env = BallistXEnv(config)
        obs, info = env.reset()

        print(f"  [OK] Custom config applied")
        print(f"  [OK] Initial distance: {info['distance']:.1f}m")
        print(f"  [OK] Max steps: {env.config.max_steps}")
        print(f"  [OK] Max accel: {env.config.missile_max_accel} m/s^2")

        env.close()
        return True
    except Exception as e:
        print(f"  [FAIL] Custom config test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_action_space():
    """Test action space bounds and scaling."""
    print("\nTesting action space...")
    try:
        from ballistx_gym.ballistx_env import BallistXEnv

        env = BallistXEnv()

        # Test minimum action
        obs, _ = env.reset()
        next_obs, reward, _, _, info = env.step(np.array([-1.0, -1.0, -1.0]))
        print(f"  [OK] Min action applied")

        # Test maximum action
        obs, _ = env.reset()
        next_obs, reward, _, _, info = env.step(np.array([1.0, 1.0, 1.0]))
        print(f"  [OK] Max action applied")

        # Test zero action
        obs, _ = env.reset()
        next_obs, reward, _, _, info = env.step(np.array([0.0, 0.0, 0.0]))
        print(f"  [OK] Zero action applied")

        env.close()
        return True
    except Exception as e:
        print(f"  [FAIL] Action space test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_observation_space():
    """Test observation space validity."""
    print("\nTesting observation space...")
    try:
        from ballistx_gym.ballistx_env import BallistXEnv

        env = BallistXEnv()
        obs, info = env.reset()

        # Check bounds
        assert env.observation_space.contains(obs), "Observation out of bounds!"
        print(f"  [OK] Observation within bounds")

        # Check shape
        expected_shape = env._calculate_obs_shape()
        assert obs.shape == expected_shape, f"Shape mismatch: {obs.shape} vs {expected_shape}"
        print(f"  [OK] Observation shape correct: {obs.shape}")

        # Check for NaN/Inf
        assert not np.any(np.isnan(obs)), "Observation contains NaN!"
        assert not np.any(np.isinf(obs)), "Observation contains Inf!"
        print(f"  [OK] Observation contains no NaN/Inf")

        env.close()
        return True
    except Exception as e:
        print(f"  [FAIL] Observation space test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_render_ansi():
    """Test ANSI rendering."""
    print("\nTesting ANSI rendering...")
    try:
        from ballistx_gym.ballistx_env import BallistXEnv

        env = BallistXEnv(render_mode='ansi')
        obs, info = env.reset()

        # Take a step
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)

        # Render
        output = env.render()
        print("  --- ANSI Output ---")
        print(output)
        print("  --- End Output ---")

        env.close()
        return True
    except Exception as e:
        print(f"  [FAIL] ANSI render failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_all_tests():
    """Run all tests."""
    print("=" * 60)
    print("BALLISTX GYMNASIUM ENVIRONMENT TEST SUITE")
    print("=" * 60)

    tests = [
        ("Import", test_import),
        ("Environment Creation", test_environment_creation),
        ("Reset", test_reset),
        ("Step", test_step),
        ("Random Agent", test_random_agent),
        ("Custom Config", test_custom_config),
        ("Action Space", test_action_space),
        ("Observation Space", test_observation_space),
        ("ANSI Render", test_render_ansi),
    ]

    results = {}
    for name, test_func in tests:
        results[name] = test_func()

    print("\n" + "=" * 60)
    print("TEST RESULTS")
    print("=" * 60)

    passed = sum(results.values())
    total = len(results)

    for name, result in results.items():
        status = "[PASS]" if result else "[FAIL]"
        print(f"{status} - {name}")

    print("=" * 60)
    print(f"Total: {passed}/{total} tests passed")
    print("=" * 60)

    return passed == total


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
