"""
BALLISTX Gymnasium Environment Package

Reinforcement Learning environments for missile guidance and ballistics simulation.

Usage:
    import sys
    sys.path.insert(0, 'path/to/ballistx')

    from ballistx_gym.ballistx_env import BallistXEnv, BallistXConfig
    from ballistx_gym.missile_guidance_env import MissileGuidanceEnv, EngagementScenario
    from ballistx_gym.discrete_ballistx_env import DiscreteBallistXEnv
"""

__version__ = "0.1.0"

# Import classes for direct access
from .ballistx_env import BallistXEnv, BallistXConfig, make_ballistx_env
from .missile_guidance_env import (
    MissileGuidanceEnv,
    EngagementScenario,
    RewardShaping,
    make_missile_guidance_env
)
from .discrete_ballistx_env import DiscreteBallistXEnv, make_discrete_ballistx_env

__all__ = [
    'BallistXEnv',
    'BallistXConfig',
    'MissileGuidanceEnv',
    'EngagementScenario',
    'RewardShaping',
    'DiscreteBallistXEnv',
    'make_ballistx_env',
    'make_missile_guidance_env',
    'make_discrete_ballistx_env',
]
