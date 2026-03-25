"""
BALLISTX Gymnasium Environment

Core RL environment that wraps the BALLISTX C++ physics engine
for missile guidance training and testing.

Author: BALLISTX Team
Version: 0.1.0
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import sys
from pathlib import Path
from typing import Optional, Tuple, Dict, Any
import warnings

# Add BALLISTX Python module to path
_ballistx_path = Path(__file__).parent.parent / "build" / "python" / "modules"
if str(_ballistx_path) not in sys.path:
    sys.path.insert(0, str(_ballistx_path))

try:
    import ballistx
except ImportError:
    warnings.warn(
        f"BALLISTX Python module not found at {_ballistx_path}. "
        "Build the project with: cmake -B build -DBUILD_PYTHON_BINDINGS=ON"
    )
    ballistx = None


class BallistXConfig:
    """Configuration for BALLISTX Gymnasium environment."""

    def __init__(
        self,
        # Simulation settings
        dt: float = 0.01,
        max_steps: int = 1000,
        max_time: float = 30.0,

        # Missile settings
        missile_pos: Tuple[float, float, float] = (0.0, 5000.0, 0.0),
        missile_vel: Tuple[float, float, float] = (300.0, 0.0, 0.0),
        missile_max_accel: float = 400.0,  # m/s²

        # Target settings
        target_pos: Tuple[float, float, float] = (8000.0, 5000.0, 0.0),
        target_vel: Tuple[float, float, float] = (200.0, 0.0, 0.0),
        target_accel: Optional[Tuple[float, float, float]] = None,

        # Guidance settings
        guidance_gain: float = 3.0,
        proximity_threshold: float = 10.0,  # meters for "hit"
        miss_distance_threshold: float = 500.0,  # meters for "miss"

        # Reward settings
        reward_distance_scale: float = 1.0,
        reward_hit_bonus: float = 1000.0,
        reward_miss_penalty: float = -500.0,
        reward_time_penalty: float = -0.1,
        reward_accel_penalty: float = -0.01,

        # Observation settings
        include_velocity: bool = True,
        include_target_state: bool = True,
        include_relative_state: bool = True,
        normalize_observations: bool = False,

        # Render settings
        render_mode: Optional[str] = None,
        render_fps: int = 30,
    ):
        # Simulation
        self.dt = dt
        self.max_steps = max_steps
        self.max_time = max_time

        # Missile
        self.missile_pos = missile_pos
        self.missile_vel = missile_vel
        self.missile_max_accel = missile_max_accel

        # Target
        self.target_pos = target_pos
        self.target_vel = target_vel
        self.target_accel = target_accel

        # Guidance
        self.guidance_gain = guidance_gain
        self.proximity_threshold = proximity_threshold
        self.miss_distance_threshold = miss_distance_threshold

        # Rewards
        self.reward_distance_scale = reward_distance_scale
        self.reward_hit_bonus = reward_hit_bonus
        self.reward_miss_penalty = reward_miss_penalty
        self.reward_time_penalty = reward_time_penalty
        self.reward_accel_penalty = reward_accel_penalty

        # Observation
        self.include_velocity = include_velocity
        self.include_target_state = include_target_state
        self.include_relative_state = include_relative_state
        self.normalize_observations = normalize_observations

        # Render
        self.render_mode = render_mode
        self.render_fps = render_fps


class BallistXEnv(gym.Env):
    """
    BALLISTX Gymnasium Environment for Missile Guidance RL.

    This environment wraps the BALLISTX C++ physics engine to provide
    a standard Gymnasium interface for training reinforcement learning
    agents on missile guidance tasks.

    **Observation Space:**
        Depends on configuration, typically includes:
        - Missile position (x, y, z)
        - Missile velocity (vx, vy, vz)
        - Target position (x, y, z)
        - Target velocity (vx, vy, vz)
        - Relative position (dx, dy, dz)
        - Relative velocity (dvx, dvy, dvz)
        - Time to go (estimated)
        - Current acceleration command (ax, ay, az)

    **Action Space:**
        Continuous 3D acceleration command (ax, ay, az) in m/s²
        Normalized to [-1, 1], scaled by max_accel

    **Reward:**
        - Distance reduction reward (positive for closing)
        - Hit bonus (+1000 for successful intercept)
        - Miss penalty (-500 for missing target)
        - Time penalty (-0.1 per step)
        - Acceleration penalty (-0.01 × |accel|)

    **Termination:**
        - Hit: distance < proximity_threshold
        - Miss: distance > miss_distance_threshold
        - Crash: missile altitude < 0

    **Truncation:**
        - Time limit: max_steps or max_time reached

    Example:
        >>> import ballistx_env
        >>> env = ballistx_env.BallistXEnv()
        >>> state, info = env.reset()
        >>> action = env.action_space.sample()
        >>> next_state, reward, terminated, truncated, info = env.step(action)
        >>> env.close()
    """

    metadata = {
        'render_modes': ['human', 'rgb_array', 'ansi'],
        'render_fps': 30
    }

    def __init__(self, config: Optional[BallistXConfig] = None, render_mode: Optional[str] = None):
        """
        Initialize BALLISTX environment.

        Args:
            config: Environment configuration (uses default if None)
            render_mode: Rendering mode ('human', 'rgb_array', 'ansi', or None)
        """
        super().__init__()

        if ballistx is None:
            raise ImportError(
                "BALLISTX Python module not available. "
                "Build with: cmake -B build -DBUILD_PYTHON_BINDINGS=ON"
            )

        # Configuration
        self.config = config if config is not None else BallistXConfig()
        if render_mode is not None:
            self.config.render_mode = render_mode

        # Simulation state
        self.current_step = 0
        self.current_time = 0.0
        self.last_distance = None
        self.terminated_reason = None

        # BALLISTX objects (initialized in reset)
        self.missile_state = None
        self.target = None
        self.guidance = None
        self.last_accel_cmd = None

        # Renderer
        self.renderer = None

        # Define action space: [ax, ay, az] normalized to [-1, 1]
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(3,),
            dtype=np.float32
        )

        # Define observation space (calculated based on config)
        obs_shape = self._calculate_obs_shape()
        obs_low = -np.inf if not self.config.normalize_observations else -1.0
        obs_high = np.inf if not self.config.normalize_observations else 1.0

        self.observation_space = spaces.Box(
            low=obs_low,
            high=obs_high,
            shape=obs_shape,
            dtype=np.float32
        )

    def _calculate_obs_shape(self) -> Tuple[int, ...]:
        """Calculate observation dimension based on config."""
        dim = 0

        # Missile state (always position)
        dim += 3  # position

        if self.config.include_velocity:
            dim += 3  # velocity

        # Target state
        if self.config.include_target_state:
            dim += 3  # position
            if self.config.include_velocity:
                dim += 3  # velocity

        # Relative state
        if self.config.include_relative_state:
            dim += 3  # relative position
            if self.config.include_velocity:
                dim += 3  # relative velocity

        # Time to go
        dim += 1

        # Last acceleration command
        dim += 3

        return (dim,)

    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """
        Reset the environment to initial state.

        Args:
            seed: Random seed for reproducibility
            options: Additional options for reset

        Returns:
            observation: Initial observation
            info: Dictionary with additional information
        """
        super().reset(seed=seed)

        # Reset simulation state
        self.current_step = 0
        self.current_time = 0.0

        # Initialize missile state
        mx, my, mz = self.config.missile_pos
        mvx, mvy, mvz = self.config.missile_vel

        self.missile_state = ballistx.State6DOF(
            ballistx.Vec3(mx, my, mz),
            ballistx.Vec3(mvx, mvy, mvz),
            ballistx.Quaternion.identity(),
            ballistx.Vec3(0.0, 0.0, 0.0)
        )

        # Initialize target
        tx, ty, tz = self.config.target_pos
        tvx, tvy, tvz = self.config.target_vel

        if self.config.target_accel is not None:
            tax, tay, taz = self.config.target_accel
            self.target = ballistx.Target(
                ballistx.Vec3(tx, ty, tz),
                ballistx.Vec3(tvx, tvy, tvz),
                ballistx.Vec3(tax, tay, taz)
            )
        else:
            self.target = ballistx.Target(
                ballistx.Vec3(tx, ty, tz),
                ballistx.Vec3(tvx, tvy, tvz)
            )

        # Initialize guidance (for baseline comparison)
        self.guidance = ballistx.ProportionalNavigation(self.config.guidance_gain)
        self.guidance.set_max_acceleration(self.config.missile_max_accel)
        self.guidance.set_proximity_threshold(self.config.proximity_threshold)

        # Initial acceleration command
        self.last_accel_cmd = np.zeros(3, dtype=np.float32)

        # Calculate initial distance
        self.last_distance = self._get_distance()

        # Reset termination reason
        self.terminated_reason = None

        # Initialize renderer if needed
        if self.config.render_mode == 'human' and self.renderer is None:
            self._init_renderer()

        # Get initial observation
        observation = self._get_obs()

        # Info dictionary
        info = self._get_info()

        return observation, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Execute one environment step.

        Args:
            action: Acceleration command [ax, ay, az] normalized to [-1, 1]

        Returns:
            observation: Next observation
            reward: Reward for this step
            terminated: True if episode ended (hit/miss/crash)
            truncated: True if episode was truncated (timeout)
            info: Additional information dictionary
        """
        # Scale action to physical acceleration
        accel_cmd = action * self.config.missile_max_accel
        self.last_accel_cmd = accel_cmd.copy()

        # Update missile physics using BALLISTX
        self._update_missile_physics(accel_cmd)

        # Update target (if it has acceleration)
        if self.config.target_accel is not None:
            self._update_target_physics()

        # Increment step and time
        self.current_step += 1
        self.current_time += self.config.dt

        # Get observation
        observation = self._get_obs()

        # Calculate reward
        reward = self._compute_reward()

        # Check termination conditions
        terminated = self._check_termination()
        truncated = self._check_truncation()

        # Get info
        info = self._get_info()

        # Render if needed
        if self.config.render_mode == 'human':
            self._render_frame()

        return observation, reward, terminated, truncated, info

    def _update_missile_physics(self, accel_cmd: np.ndarray):
        """Update missile state using BALLISTX physics with gravity."""
        # Current state
        pos = self.missile_state.get_position()
        vel = self.missile_state.get_velocity()

        # Apply guidance acceleration
        ax, ay, az = accel_cmd
        guidance_accel = ballistx.Vec3(ax, ay, az)

        # Add gravity (crucial! without this, missile will fall)
        gravity = ballistx.Vec3(0.0, -9.80665, 0.0)
        total_accel = guidance_accel + gravity

        # Semi-implicit Euler integration (symplectic - better energy conservation)
        # Update velocity first, then use new velocity for position
        new_vel = vel + total_accel * self.config.dt
        new_pos = pos + new_vel * self.config.dt  # Use new_vel!

        self.missile_state.set_position(new_pos)
        self.missile_state.set_velocity(new_vel)

    def _update_target_physics(self):
        """Update target state if it has acceleration."""
        if self.config.target_accel is None:
            return

        tax, tay, taz = self.config.target_accel
        accel = ballistx.Vec3(tax, tay, taz)

        vel = self.target.velocity
        pos = self.target.position

        new_vel = vel + accel * self.config.dt
        new_pos = pos + vel * self.config.dt

        self.target.position = new_pos
        self.target.velocity = new_vel

    def _get_obs(self) -> np.ndarray:
        """Construct observation vector."""
        obs_list = []

        # Missile state
        m_pos = self.missile_state.get_position()
        obs_list.extend([m_pos.x, m_pos.y, m_pos.z])

        if self.config.include_velocity:
            m_vel = self.missile_state.get_velocity()
            obs_list.extend([m_vel.x, m_vel.y, m_vel.z])

        # Target state
        if self.config.include_target_state:
            t_pos = self.target.position
            t_vel = self.target.velocity
            obs_list.extend([t_pos.x, t_pos.y, t_pos.z])

            if self.config.include_velocity:
                obs_list.extend([t_vel.x, t_vel.y, t_vel.z])

        # Relative state
        if self.config.include_relative_state:
            rel_pos = t_pos - m_pos
            obs_list.extend([rel_pos.x, rel_pos.y, rel_pos.z])

            if self.config.include_velocity:
                m_vel = self.missile_state.get_velocity()
                rel_vel = t_vel - m_vel
                obs_list.extend([rel_vel.x, rel_vel.y, rel_vel.z])

        # Time to go (estimated)
        distance = self._get_distance()
        closing_speed = self._get_closing_speed()
        time_to_go = distance / (closing_speed + 1e-6) if closing_speed > 0 else 999.0
        obs_list.append(min(time_to_go, 100.0))  # Cap at 100s

        # Last acceleration command (normalized)
        obs_list.extend(self.last_accel_cmd / self.config.missile_max_accel)

        observation = np.array(obs_list, dtype=np.float32)

        # Optional normalization
        if self.config.normalize_observations:
            observation = self._normalize_observation(observation)

        return observation

    def _normalize_observation(self, obs: np.ndarray) -> np.ndarray:
        """Normalize observations to [-1, 1] range (simplified)."""
        # This is a simplified normalization - proper version would use
        # environment-specific scaling factors
        return np.tanh(obs / 10000.0)  # Rough scaling

    def _compute_reward(self) -> float:
        """Calculate reward for current step."""
        current_distance = self._get_distance()

        # Distance reduction reward (positive for closing)
        if self.last_distance is not None:
            distance_delta = self.last_distance - current_distance
            reward = distance_delta * self.config.reward_distance_scale
        else:
            reward = 0.0

        # Update last distance
        self.last_distance = current_distance

        # Hit bonus
        if current_distance < self.config.proximity_threshold:
            reward += self.config.reward_hit_bonus

        # Miss penalty (when moving away)
        if current_distance > self.config.miss_distance_threshold:
            reward += self.config.reward_miss_penalty

        # Time penalty (encourage faster engagement)
        reward += self.config.reward_time_penalty

        # Acceleration penalty (energy efficiency)
        accel_magnitude = np.linalg.norm(self.last_accel_cmd)
        reward += self.config.reward_accel_penalty * accel_magnitude

        return float(reward)

    def _check_termination(self) -> bool:
        """Check if episode should terminate."""
        distance = self._get_distance()
        altitude = self.missile_state.get_position().y

        # Hit condition
        if distance < self.config.proximity_threshold:
            self.terminated_reason = 'hit'
            return True

        # Miss condition (too far)
        if distance > self.config.miss_distance_threshold:
            self.terminated_reason = 'miss'
            return True

        # Crash condition
        if altitude < 0:
            self.terminated_reason = 'crash'
            return True

        return False

    def _check_truncation(self) -> bool:
        """Check if episode should be truncated."""
        # Step limit
        if self.current_step >= self.config.max_steps:
            self.terminated_reason = 'max_steps'
            return True

        # Time limit
        if self.current_time >= self.config.max_time:
            self.terminated_reason = 'timeout'
            return True

        return False

    def _get_distance(self) -> float:
        """Get distance from missile to target."""
        m_pos = self.missile_state.get_position()
        t_pos = self.target.position
        diff = t_pos - m_pos
        return diff.magnitude()

    def _get_closing_speed(self) -> float:
        """Get closing speed (positive = approaching)."""
        m_pos = self.missile_state.get_position()
        m_vel = self.missile_state.get_velocity()
        t_pos = self.target.position
        t_vel = self.target.velocity

        rel_pos = (t_pos - m_pos).normalized()
        rel_vel = t_vel - m_vel
        closing_speed = -rel_vel.dot(rel_pos)  # Negative because we want approach

        return closing_speed

    def _get_info(self) -> Dict:
        """Construct info dictionary."""
        distance = self._get_distance()
        closing_speed = self._get_closing_speed()

        # Get baseline guidance command for comparison
        baseline_cmd = self.guidance.calculate_command(self.missile_state, self.target)

        # PN guidance produces lateral acceleration commands.
        # For fair comparison in environment that adds gravity, we need to add
        # gravity compensation to the baseline so PN can properly counteract gravity.
        # This makes PN's effective total acceleration match real missile behavior.
        GRAVITY = 9.80665
        baseline_accel_with_gravity = np.array([
            baseline_cmd.acceleration_command.x,
            baseline_cmd.acceleration_command.y + GRAVITY,  # Gravity compensation
            baseline_cmd.acceleration_command.z
        ], dtype=np.float32)

        return {
            'distance': float(distance),
            'closing_speed': float(closing_speed),
            'time_to_go': float(distance / (abs(closing_speed) + 1e-6)),
            'current_time': float(self.current_time),
            'current_step': int(self.current_step),
            'terminated_reason': self.terminated_reason,
            'baseline_accel': baseline_accel_with_gravity,
            'is_hit': distance < self.config.proximity_threshold,
            'missile_position': np.array([
                self.missile_state.get_position().x,
                self.missile_state.get_position().y,
                self.missile_state.get_position().z
            ], dtype=np.float32),
            'target_position': np.array([
                self.target.position.x,
                self.target.position.y,
                self.target.position.z
            ], dtype=np.float32),
        }

    def render(self):
        """Render the environment."""
        if self.config.render_mode == 'human':
            self._render_frame()
        elif self.config.render_mode == 'rgb_array':
            return self._render_frame(return_array=True)
        elif self.config.render_mode == 'ansi':
            return self._render_ansi()

    def _init_renderer(self):
        """Initialize the renderer."""
        # Lazy import to avoid hard dependency
        try:
            from .renderers.trajectory_2d import TrajectoryRenderer2D
            self.renderer = TrajectoryRenderer2D()
        except ImportError:
            warnings.warn("Renderer not available. Install matplotlib.")
            self.renderer = None

    def _render_frame(self, return_array=False):
        """Render a single frame."""
        if self.renderer is not None:
            return self.renderer.render(
                self.missile_state.get_position(),
                self.target.position,
                return_array=return_array
            )
        else:
            # Fallback: print to console
            if self.current_step % 10 == 0:
                dist = self._get_distance()
                print(f"Step {self.current_step}: Distance = {dist:.1f}m")

    def _render_ansi(self) -> str:
        """Render ASCII representation."""
        m_pos = self.missile_state.get_position()
        t_pos = self.target.position
        distance = self._get_distance()

        output = [
            f"Step: {self.current_step}",
            f"Time: {self.current_time:.2f}s",
            f"Missile: ({m_pos.x:.1f}, {m_pos.y:.1f}, {m_pos.z:.1f})",
            f"Target:  ({t_pos.x:.1f}, {t_pos.y:.1f}, {t_pos.z:.1f})",
            f"Distance: {distance:.1f}m",
            f"Reason: {self.terminated_reason or 'active'}"
        ]

        return "\n".join(output)

    def close(self):
        """Clean up environment resources."""
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


# Convenience function to create environment with default config
def make_ballistx_env(**kwargs) -> BallistXEnv:
    """
    Create BALLISTX environment with custom configuration.

    Args:
        **kwargs: Arguments passed to BallistXConfig

    Returns:
        Configured BALLISTX environment

    Example:
        >>> env = make_ballistx_env(
        ...     missile_pos=(0, 5000, 0),
        ...     target_pos=(10000, 5000, 0),
        ...     max_steps=500
        ... )
    """
    config = BallistXConfig(**kwargs)
    return BallistXEnv(config)
