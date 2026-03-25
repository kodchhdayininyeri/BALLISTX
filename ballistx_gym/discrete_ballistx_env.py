"""
Discrete Action Version of BALLISTX Environment for DQN Training

DQN requires discrete action spaces. This environment provides a discrete
approximation of continuous acceleration commands.
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Optional, Tuple, Dict
from .ballistx_env import BallistXEnv, BallistXConfig


class DiscreteBallistXEnv(BallistXEnv):
    """
    Discrete action version of BALLISTX environment for DQN training.

    Instead of continuous acceleration commands, this environment provides
    a discrete set of actions representing different guidance maneuvers.

    Actions:
        0: No acceleration (coast)
        1: Accelerate forward
        2: Accelerate left
        3: Accelerate right
        4: Accelerate up
        5: Accelerate down
        6: Accelerate forward-left
        7: Accelerate forward-right
        8: Accelerate forward-up
        9: Accelerate forward-down
    """

    metadata = {
        'render_modes': ['human', 'rgb_array', 'ansi'],
        'render_fps': 30
    }

    # Discrete action definitions
    # Each action maps to a normalized acceleration vector [ax, ay, az]
    ACTIONS = {
        0: np.array([0.0, 0.0, 0.0]),      # Coast
        1: np.array([1.0, 0.0, 0.0]),      # Forward
        2: np.array([0.0, 0.0, 0.5]),      # Left
        3: np.array([0.0, 0.0, -0.5]),     # Right
        4: np.array([0.0, 0.5, 0.0]),      # Up
        5: np.array([0.0, -0.5, 0.0]),     # Down
        6: np.array([1.0, 0.0, 0.3]),      # Forward-Left
        7: np.array([1.0, 0.0, -0.3]),     # Forward-Right
        8: np.array([1.0, 0.3, 0.0]),      # Forward-Up
        9: np.array([1.0, -0.3, 0.0]),     # Forward-Down
    }

    ACTION_NAMES = {
        0: "Coast",
        1: "Forward",
        2: "Left",
        3: "Right",
        4: "Up",
        5: "Down",
        6: "Forward-Left",
        7: "Forward-Right",
        8: "Forward-Up",
        9: "Forward-Down",
    }

    def __init__(self, config: Optional[BallistXConfig] = None, render_mode: Optional[str] = None):
        # Initialize parent
        super().__init__(config=config, render_mode=render_mode)

        # Override action space with discrete actions
        self.action_space = spaces.Discrete(len(self.ACTIONS))

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Execute one environment step with discrete action.

        Args:
            action: Discrete action index [0, n_actions-1]

        Returns:
            Same as parent step()
        """
        # Map discrete action to continuous acceleration
        continuous_action = self.ACTIONS[action]

        # Call parent step with continuous action
        return super().step(continuous_action)

    def get_action_name(self, action: int) -> str:
        """Get human-readable action name."""
        return self.ACTION_NAMES.get(action, f"Unknown({action})")


def make_discrete_ballistx_env(**kwargs) -> DiscreteBallistXEnv:
    """
    Convenience function to create discrete BALLISTX environment.

    Args:
        **kwargs: Arguments passed to BallistXConfig

    Returns:
        Configured DiscreteBallistXEnv

    Example:
        >>> env = make_discrete_ballistx_env(
        ...     missile_pos=(0, 5000, 0),
        ...     target_pos=(10000, 5000, 0)
        ... )
    """
    config = BallistXConfig(**kwargs)
    return DiscreteBallistXEnv(config=config)
