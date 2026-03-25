"""
2D Trajectory Renderer for BALLISTX Gymnasium Environment

Uses matplotlib for real-time visualization of missile-target engagement.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path

try:
    import ballistx
except ImportError:
    ballistx = None


class TrajectoryRenderer2D:
    """
    2D trajectory renderer using matplotlib.

    Shows top-down (X-Z) and side (X-Y) views of the engagement.
    """

    def __init__(self, figsize=(12, 6), fps=30):
        """
        Initialize renderer.

        Args:
            figsize: Figure size (width, height)
            fps: Frames per second for animation
        """
        self.figsize = figsize
        self.fps = fps

        # Create figure
        self.fig, (self.ax_top, self.ax_side) = plt.subplots(1, 2, figsize=figsize)

        # Trajectory history
        self.missile_trail_x = []
        self.missile_trail_y = []
        self.missile_trail_z = []
        self.target_trail_x = []
        self.target_trail_y = []
        self.target_trail_z = []

        # Plot objects
        self.missile_top = None
        self.target_top = None
        self.missile_trail_top = None
        self.target_trail_top = None

        self.missile_side = None
        self.target_side = None
        self.missile_trail_side = None
        self.target_trail_side = None

        self._setup_plots()

    def _setup_plots(self):
        """Setup plot layouts."""
        # Top view (X-Z plane)
        self.ax_top.set_title("Top View (X-Z Plane)")
        self.ax_top.set_xlabel("Range X (m)")
        self.ax_top.set_ylabel("Cross-range Z (m)")
        self.ax_top.grid(True, alpha=0.3)
        self.ax_top.set_aspect('equal')

        # Initialize plot objects
        self.missile_top, = self.ax_top.plot([], [], 'b>', markersize=10, label='Missile')
        self.target_top, = self.ax_top.plot([], [], 'rs', markersize=12, label='Target')
        self.missile_trail_top, = self.ax_top.plot([], [], 'b-', alpha=0.5, linewidth=1)
        self.target_trail_top, = self.ax_top.plot([], [], 'r--', alpha=0.5, linewidth=1)
        self.ax_top.legend()

        # Side view (X-Y plane)
        self.ax_side.set_title("Side View (X-Y Plane)")
        self.ax_side.set_xlabel("Range X (m)")
        self.ax_side.set_ylabel("Altitude Y (m)")
        self.ax_side.grid(True, alpha=0.3)
        self.ax_side.set_aspect('equal')

        # Initialize plot objects
        self.missile_side, = self.ax_side.plot([], [], 'b>', markersize=10, label='Missile')
        self.target_side, = self.ax_side.plot([], [], 'rs', markersize=12, label='Target')
        self.missile_trail_side, = self.ax_side.plot([], [], 'b-', alpha=0.5, linewidth=1)
        self.target_trail_side, = self.ax_side.plot([], [], 'r--', alpha=0.5, linewidth=1)
        self.ax_side.legend(loc='upper right')

        plt.tight_layout()

    def render(self, missile_pos, target_pos, return_array=False):
        """
        Render current frame.

        Args:
            missile_pos: BallistX Vec3 or numpy array [x, y, z]
            target_pos: BallistX Vec3 or numpy array [x, y, z]
            return_array: If True, return RGB array instead of displaying

        Returns:
            RGB array if return_array=True, None otherwise
        """
        # Convert to numpy if BallistX Vec3
        if hasattr(missile_pos, 'x'):
            mx, my, mz = missile_pos.x, missile_pos.y, missile_pos.z
        else:
            mx, my, mz = missile_pos

        if hasattr(target_pos, 'x'):
            tx, ty, tz = target_pos.x, target_pos.y, target_pos.z
        else:
            tx, ty, tz = target_pos

        # Update trails
        self.missile_trail_x.append(mx)
        self.missile_trail_y.append(my)
        self.missile_trail_z.append(mz)
        self.target_trail_x.append(tx)
        self.target_trail_y.append(ty)
        self.target_trail_z.append(tz)

        # Limit trail length
        max_trail = 500
        if len(self.missile_trail_x) > max_trail:
            self.missile_trail_x = self.missile_trail_x[-max_trail:]
            self.missile_trail_y = self.missile_trail_y[-max_trail:]
            self.missile_trail_z = self.missile_trail_z[-max_trail:]
            self.target_trail_x = self.target_trail_x[-max_trail:]
            self.target_trail_y = self.target_trail_y[-max_trail:]
            self.target_trail_z = self.target_trail_z[-max_trail:]

        # Update plot data
        self.missile_top.set_data([mx], [mz])
        self.target_top.set_data([tx], [tz])
        self.missile_trail_top.set_data(self.missile_trail_x, self.missile_trail_z)
        self.target_trail_top.set_data(self.target_trail_x, self.target_trail_z)

        self.missile_side.set_data([mx], [my])
        self.target_side.set_data([tx], [ty])
        self.missile_trail_side.set_data(self.missile_trail_x, self.missile_trail_y)
        self.target_trail_side.set_data(self.target_trail_x, self.target_trail_y)

        # Auto-scale axes
        self._autoscale_axes()

        if return_array:
            # Convert to RGB array
            self.fig.canvas.draw()
            width, height = self.fig.get_size_inches() * self.fig.dpi
            image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(int(height), int(width), 3)
            return image
        else:
            plt.pause(0.001)
            return None

    def _autoscale_axes(self):
        """Auto-scale axes to fit all data."""
        all_x = self.missile_trail_x + self.target_trail_x
        all_y = self.missile_trail_y + self.target_trail_y
        all_z = self.missile_trail_z + self.target_trail_z

        if not all_x:
            return

        margin = 500  # meters

        self.ax_top.set_xlim(min(all_x) - margin, max(all_x) + margin)
        self.ax_top.set_ylim(min(all_z) - margin, max(all_z) + margin)

        self.ax_side.set_xlim(min(all_x) - margin, max(all_x) + margin)
        self.ax_side.set_ylim(min(all_y) - margin, max(all_y) + margin)

    def reset(self):
        """Reset trails and plots."""
        self.missile_trail_x = []
        self.missile_trail_y = []
        self.missile_trail_z = []
        self.target_trail_x = []
        self.target_trail_y = []
        self.target_trail_z = []

    def close(self):
        """Close figure."""
        plt.close(self.fig)
