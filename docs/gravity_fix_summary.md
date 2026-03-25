# Gravity Fix Summary

## Problem Discovered

The user identified that the missile was crashing into the ground with strange trajectories. The root cause was **missing gravity** in the physics update function.

## Fixes Applied

### 1. Added Gravity to Physics Update (ballistx_env.py:372-391)

**Before:**
```python
def _update_missile_physics(self, accel_cmd: np.ndarray):
    """Update missile state using BALLISTX physics with gravity."""
    pos = self.missile_state.get_position()
    vel = self.missile_state.get_velocity()

    ax, ay, az = accel_cmd
    guidance_accel = ballistx.Vec3(ax, ay, az)

    # MISSING: No gravity compensation!

    new_vel = vel + guidance_accel * self.config.dt
    new_pos = pos + vel * self.config.dt
```

**After:**
```python
def _update_missile_physics(self, accel_cmd: np.ndarray):
    """Update missile state using BALLISTX physics with gravity."""
    pos = self.missile_state.get_position()
    vel = self.missile_state.get_velocity()

    ax, ay, az = accel_cmd
    guidance_accel = ballistx.Vec3(ax, ay, az)

    # Add gravity (crucial! without this, missile will fall)
    gravity = ballistx.Vec3(0.0, -9.80665, 0.0)
    total_accel = guidance_accel + gravity  # <-- FIXED

    new_vel = vel + total_accel * self.config.dt
    new_pos = pos + vel * self.config.dt
```

### 2. Added Gravity Compensation for PN Baseline (ballistx_env.py:551-582)

The PN guidance in BALLISTX C++ code produces **lateral acceleration** commands (perpendicular to velocity), not total acceleration. For fair comparison, we need to add gravity compensation:

```python
def _get_info(self) -> Dict:
    """Construct info dictionary."""
    # ...
    baseline_cmd = self.guidance.calculate_command(self.missile_state, self.target)

    # PN guidance produces lateral acceleration commands.
    # For fair comparison in environment that adds gravity, we need to add
    # gravity compensation to the baseline so PN can properly counteract gravity.
    GRAVITY = 9.80665
    baseline_accel_with_gravity = np.array([
        baseline_cmd.acceleration_command.x,
        baseline_cmd.acceleration_command.y + GRAVITY,  # Gravity compensation
        baseline_cmd.acceleration_command.z
    ], dtype=np.float32)
    # ...
```

### 3. Fixed Scenario to be Achievable

Updated all training/comparison scenarios to use achievable parameters:
- **Missile speed:** 500 m/s (was 350 m/s)
- **Target speed:** 100 m/s (was 200 m/s)
- **Closing speed:** 400 m/s (was 150 m/s)
- **Max time:** 20s (max_steps=1000, dt=0.02)

With these parameters, the missile can cover 8000m in exactly 20s.

## Results

### Before Fix (Broken Scenario)
```
METRIC                    RANDOM AGENT         PN GUIDANCE          WINNER
--------------------------------------------------------------------------------
Avg Reward                    1430.5              3401.0          PN
Avg Final Distance (m)        4594.9              4500.0          PN
```

### After Fix (Achievable Scenario)
```
METRIC                    RANDOM AGENT         PN GUIDANCE          WINNER
--------------------------------------------------------------------------------
Avg Reward                    -186.0              8831.7          PN
Avg Final Distance (m)        4237.7                10.0          PN
--------------------------------------------------------------------------------
[PERFORMANCE GAP]
   Reward Improvement: +4847.1%
   Distance Improvement: +99.8% (closer is better)
```

## Key Insights

1. **PN guidance now properly intercepts** (10m final distance vs 15m proximity threshold)
2. **Random agent performs worse** because gravity pulls it down when it doesn't compensate
3. **The RL agent has a clear learning target**: Beat 8832 reward to match PN

## Files Modified

1. `ballistx_gym/ballistx_env.py` - Gravity physics + PN baseline compensation
2. `examples/train_dqn.py` - Achievable scenario parameters
3. `examples/train_ppo.py` - Achievable scenario parameters
4. `examples/compare_random_vs_pn.py` - Achievable scenario parameters

## Next Steps

1. **Re-train DQN model** with corrected physics - previous model was trained with broken physics
2. **Train PPO model** from scratch with achievable scenario
3. **Compare trained agents against PN baseline** to verify RL can learn optimal guidance
