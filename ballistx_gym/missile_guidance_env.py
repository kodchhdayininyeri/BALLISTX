"""
Missile Guidance Environment - Specialized BALLISTX RL Environment

Extended environment with advanced features for missile guidance training:
- Multiple engagement scenarios
- Maneuvering targets
- Reward shaping options
- State normalization
- Curriculum learning support
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Optional, Tuple, Dict, Any, List
from enum import Enum

from .ballistx_env import BallistXEnv, BallistXConfig


class EngagementScenario(Enum):
    """Predefined engagement scenarios for training."""

    HEAD_ON = "head_on"
    """Missile and target approaching each other"""

    TAIL_CHASE = "tail_chase"
    """Missile pursuing target from behind"""

    CROSSING = "crossing"
    """Target crossing perpendicular to missile path"""

    MANEUVERING = "maneuvering"
    """Target performing evasive maneuvers"""

    STATIONARY = "stationary"
    """Stationary target (ground attack)"""


class RewardShaping(Enum):
    """Reward shaping options."""

    DISTANCE_ONLY = "distance_only"
    """Only reward based on distance reduction"""

    HIT_MISS_PENALTY = "hit_miss_penalty"
    """Distance reward + hit bonus + miss penalty"""

    SHAPED = "shaped"
    """Includes energy penalty, time penalty, acceleration penalty"""

    DENSE = "dense"
    """Continuous feedback with intermediate goals"""


class MissileGuidanceEnv(BallistXEnv):
    """
    Specialized environment for missile guidance RL training.

    Extends BallistXEnv with:
    - Multiple engagement scenarios
    - Configurable reward shaping
    - Maneuvering targets with different patterns
    - Curriculum learning support
    - Episode difficulty indicators

    Example:
        >>> env = MissileGuidanceEnv(scenario=EngagementScenario.HEAD_ON)
        >>> env.set_difficulty(0.5)  # Medium difficulty
        >>> obs, info = env.reset()
    """

    metadata = {
        'render_modes': ['human', 'rgb_array', 'ansi'],
        'render_fps': 30
    }

    # Scenario definitions
    SCENARIOS = {
        EngagementScenario.HEAD_ON: {
            'missile_pos': (0.0, 5000.0, 0.0),
            'missile_vel': (400.0, 0.0, 0.0),
            'target_pos': (10000.0, 5000.0, 0.0),
            'target_vel': (-250.0, 0.0, 0.0),
            'target_accel': None,
        },
        EngagementScenario.TAIL_CHASE: {
            'missile_pos': (0.0, 5000.0, 0.0),
            'missile_vel': (350.0, 0.0, 0.0),
            'target_pos': (8000.0, 5000.0, 0.0),
            'target_vel': (200.0, 0.0, 0.0),
            'target_accel': None,
        },
        EngagementScenario.CROSSING: {
            'missile_pos': (0.0, 5000.0, -5000.0),
            'missile_vel': (400.0, 0.0, 50.0),
            'target_pos': (8000.0, 5000.0, 0.0),
            'target_vel': (200.0, 0.0, -100.0),
            'target_accel': None,
        },
        EngagementScenario.MANEUVERING: {
            'missile_pos': (0.0, 5000.0, 0.0),
            'missile_vel': (400.0, 0.0, 0.0),
            'target_pos': (8000.0, 5000.0, 0.0),
            'target_vel': (250.0, 0.0, 0.0),
            'target_accel': (0.0, 30.0, 0.0),  # Pulling up
        },
        EngagementScenario.STATIONARY: {
            'missile_pos': (0.0, 3000.0, 0.0),
            'missile_vel': (250.0, 0.0, 0.0),
            'target_pos': (5000.0, 0.0, 0.0),
            'target_vel': (0.0, 0.0, 0.0),
            'target_accel': None,
        },
    }

    def __init__(
        self,
        scenario: EngagementScenario = EngagementScenario.TAIL_CHASE,
        reward_shaping: RewardShaping = RewardShaping.SHAPED,
        difficulty: float = 0.5,
        **kwargs
    ):
        """
        Initialize missile guidance environment.

        Args:
            scenario: Engagement scenario type
            reward_shaping: Reward shaping method
            difficulty: Difficulty level [0.0, 1.0]
            **kwargs: Additional arguments passed to BallistXConfig
        """
        # Get scenario configuration
        scenario_config = self.SCENARIOS[scenario]
        config = BallistXConfig(**scenario_config, **kwargs)

        # Apply difficulty scaling
        self._apply_difficulty(config, difficulty)

        # Store settings
        self.scenario = scenario
        self.reward_shaping = reward_shaping
        self.difficulty = difficulty

        # Maneuvering state
        self.maneuver_time = 0.0
        self.maneuver_pattern = None

        # Initialize parent
        super().__init__(config=config)

        # Override action space to allow discrete action options
        self._setup_action_options()

    def _apply_difficulty(self, config: BallistXConfig, difficulty: float):
        """
        Apply difficulty scaling to configuration.

        Difficulty affects:
        - 0.0 (Easy): Stationary/slow targets, large hit threshold
        - 0.5 (Medium): Moving targets, moderate hit threshold
        - 1.0 (Hard): Fast maneuvering targets, tight hit threshold
        """
        # Scale target speed
        base_target_vel = np.array(config.target_vel)
        speed_multiplier = 1.0 + difficulty * 0.5
        scaled_vel = tuple(base_target_vel * speed_multiplier)
        config.target_vel = scaled_vel

        # Scale hit threshold (harder = smaller threshold)
        config.proximity_threshold = 20.0 * (1.0 - difficulty * 0.5)

        # Scale max acceleration (harder = more constrained)
        config.missile_max_accel = 500.0 * (1.0 - difficulty * 0.2)

    def _setup_action_options(self):
        """Setup action options (continuous, discrete, or hybrid)."""
        # For now, keep continuous action space
        # Future: Add discrete action options for DQN
        pass

    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment with scenario-specific initialization."""
        obs, info = super().reset(seed=seed, options=options)

        # Reset maneuver state
        self.maneuver_time = 0.0

        # Add scenario info
        info['scenario'] = self.scenario.value
        info['difficulty'] = self.difficulty
        info['reward_shaping'] = self.reward_shaping.value

        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute step with scenario-specific logic."""
        # Update maneuvering target if applicable
        if self.scenario == EngagementScenario.MANEUVERING:
            self._update_maneuver()

        # Call parent step
        obs, reward, terminated, truncated, info = super().step(action)

        # Apply reward shaping
        reward = self._shape_reward(reward, info)

        # Add scenario info
        info['maneuver_accel'] = getattr(self, '_maneuver_accel', None)

        return obs, reward, terminated, truncated, info

    def _update_maneuver(self):
        """Update target maneuver for maneuvering scenario."""
        self.maneuver_time += self.config.dt

        # Simple weave maneuver pattern
        # Target alternates between pulling up and pushing down
        maneuver_period = 4.0  # seconds
        accel_magnitude = 40.0  # m/s²

        accel_y = accel_magnitude * np.sin(2 * np.pi * self.maneuver_time / maneuver_period)

        # Update target acceleration
        self.config.target_accel = (0.0, accel_y, 0.0)
        self._maneuver_accel = accel_y

    def _shape_reward(self, base_reward: float, info: Dict) -> float:
        """
        Apply reward shaping based on configuration.

        Args:
            base_reward: Base reward from environment
            info: Info dictionary with episode state

        Returns:
            Shaped reward
        """
        distance = info['distance']
        closing_speed = info['closing_speed']

        if self.reward_shaping == RewardShaping.DISTANCE_ONLY:
            # Only distance-based reward
            return base_reward

        elif self.reward_shaping == RewardShaping.HIT_MISS_PENALTY:
            # Distance + hit bonus + miss penalty
            reward = base_reward
            if distance < self.config.proximity_threshold:
                reward += self.config.reward_hit_bonus
            if distance > self.config.miss_distance_threshold:
                reward += self.config.reward_miss_penalty
            return reward

        elif self.reward_shaping == RewardShaping.SHAPED:
            # Full shaped reward (default from parent)
            return base_reward

        elif self.reward_shaping == RewardShaping.DENSE:
            # Dense reward with intermediate goals
            reward = base_reward

            # Bonus for closing fast
            if closing_speed > 100:
                reward += 1.0

            # Bonus for being within certain ranges
            if distance < 1000:
                reward += 10.0
            if distance < 500:
                reward += 20.0
            if distance < 100:
                reward += 50.0

            return reward

        return base_reward

    def set_difficulty(self, difficulty: float):
        """
        Update difficulty level.

        Args:
            difficulty: New difficulty [0.0, 1.0]
        """
        self.difficulty = np.clip(difficulty, 0.0, 1.0)
        self._apply_difficulty(self.config, self.difficulty)

    def set_scenario(self, scenario: EngagementScenario):
        """
        Change engagement scenario.

        Note: Requires calling reset() to take effect.
        """
        self.scenario = scenario
        scenario_config = self.SCENARIOS[scenario]

        # Update config
        for key, value in scenario_config.items():
            setattr(self.config, key, value)

        # Re-apply difficulty
        self._apply_difficulty(self.config, self.difficulty)

    def get_scenario_info(self) -> Dict:
        """Get information about current scenario."""
        return {
            'scenario': self.scenario.value,
            'difficulty': self.difficulty,
            'reward_shaping': self.reward_shaping.value,
            'config': {
                'missile_pos': self.config.missile_pos,
                'missile_vel': self.config.missile_vel,
                'target_pos': self.config.target_pos,
                'target_vel': self.config.target_vel,
                'max_accel': self.config.missile_max_accel,
            }
        }


def make_missile_guidance_env(
    scenario: str = 'tail_chase',
    reward_shaping: str = 'shaped',
    difficulty: float = 0.5,
    **kwargs
) -> MissileGuidanceEnv:
    """
    Convenience function to create missile guidance environment.

    Args:
        scenario: Scenario name ('head_on', 'tail_chase', 'crossing', 'maneuvering', 'stationary')
        reward_shaping: Reward shaping method
        difficulty: Difficulty level [0.0, 1.0]
        **kwargs: Additional arguments

    Returns:
        Configured MissileGuidanceEnv

    Example:
        >>> env = make_missile_guidance_env(
        ...     scenario='head_on',
        ...     difficulty=0.7,
        ...     render_mode='human'
        ... )
    """
    scenario_enum = EngagementScenario(scenario)
    reward_enum = RewardShaping(reward_shaping)

    return MissileGuidanceEnv(
        scenario=scenario_enum,
        reward_shaping=reward_enum,
        difficulty=difficulty,
        **kwargs
    )
