#!/usr/bin/env python3
"""
Kalman Filter Visualization Demo

This script visualizes the Extended Kalman Filter tracking performance
by comparing:
- True projectile trajectory
- Noisy radar measurements
- Kalman filter estimates
- Position uncertainty over time

Requirements:
    pip install plotly pandas kaleido

Output:
    - kalman_visualization.html (interactive 3D plot)
    - kalman_metrics.html (error analysis plots)
"""

import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
from pathlib import Path


def load_data():
    """Load tracking data from CSV files."""
    trajectory_df = pd.read_csv('kalman_trajectory.csv')
    measurements_df = pd.read_csv('kalman_measurements.csv')
    errors_df = pd.read_csv('kalman_errors.csv')
    return trajectory_df, measurements_df, errors_df


def create_3d_trajectory_plot(trajectory_df, measurements_df):
    """Create 3D trajectory visualization."""
    fig = go.Figure()

    # True trajectory
    fig.add_trace(go.Scatter3d(
        x=trajectory_df['true_x'],
        y=trajectory_df['true_y'],
        z=trajectory_df['true_z'],
        mode='lines',
        name='True Trajectory',
        line=dict(color='green', width=4),
        hovertemplate='<b>True Position</b><br>' +
                     'X: %{x:.1f} m<br>' +
                     'Y: %{y:.1f} m<br>' +
                     'Z: %{z:.1f} m<br>' +
                     'Time: %{text:.2f} s',
        text=trajectory_df['time']
    ))

    # Estimated trajectory
    fig.add_trace(go.Scatter3d(
        x=trajectory_df['est_x'],
        y=trajectory_df['est_y'],
        z=trajectory_df['est_z'],
        mode='lines',
        name='Kalman Estimate',
        line=dict(color='blue', width=3, dash='dash'),
        hovertemplate='<b>Estimated Position</b><br>' +
                     'X: %{x:.1f} m<br>' +
                     'Y: %{y:.1f} m<br>' +
                     'Z: %{z:.1f} m<br>' +
                     'Uncertainty: %{text:.2f} m',
        text=trajectory_df['uncertainty']
    ))

    # Uncertainty ellipsoid (show every 10th point)
    skip = 10
    for i in range(0, len(trajectory_df), skip):
        unc = trajectory_df['uncertainty'].iloc[i]
        fig.add_trace(go.Scatter3d(
            x=[trajectory_df['est_x'].iloc[i]],
            y=[trajectory_df['est_y'].iloc[i]],
            z=[trajectory_df['est_z'].iloc[i]],
            mode='markers',
            marker=dict(
                size=unc / 5,
                color='blue',
                opacity=0.1,
                symbol='circle'
            ),
            showlegend=False,
            hoverinfo='skip'
        ))

    # Start position
    fig.add_trace(go.Scatter3d(
        x=[trajectory_df['true_x'].iloc[0]],
        y=[trajectory_df['true_y'].iloc[0]],
        z=[trajectory_df['true_z'].iloc[0]],
        mode='markers',
        name='Launch Point',
        marker=dict(size=10, color='lime', symbol='diamond'),
        hovertemplate='<b>Launch Point</b><br>' +
                     'X: %{x:.1f} m<br>' +
                     'Y: %{y:.1f} m<br>' +
                     'Z: %{z:.1f} m'
    ))

    # Impact point
    fig.add_trace(go.Scatter3d(
        x=[trajectory_df['true_x'].iloc[-1]],
        y=[trajectory_df['true_y'].iloc[-1]],
        z=[trajectory_df['true_z'].iloc[-1]],
        mode='markers',
        name='Impact Point',
        marker=dict(size=10, color='red', symbol='x'),
        hovertemplate='<b>Impact Point</b><br>' +
                     'X: %{x:.1f} m<br>' +
                     'Y: %{y:.1f} m<br>' +
                     'Z: %{z:.1f} m'
    ))

    # Estimated position markers (every 50th point)
    for i in range(0, len(trajectory_df), 50):
        fig.add_trace(go.Scatter3d(
            x=[trajectory_df['est_x'].iloc[i]],
            y=[trajectory_df['est_y'].iloc[i]],
            z=[trajectory_df['est_z'].iloc[i]],
            mode='markers',
            marker=dict(size=4, color='blue', opacity=0.6),
            showlegend=False,
            hovertemplate='<b>t = ' + f'{trajectory_df["time"].iloc[i]:.2f} s</b><br>' +
                         'Est: (%{x:.1f}, %{y:.1f}, %{z:.1f}) m<br>' +
                         'Uncertainty: ' + f'{trajectory_df["uncertainty"].iloc[i]:.2f} m'
        ))

    # Ground plane
    max_range = max(
        trajectory_df['true_x'].max(),
        trajectory_df['true_y'].max(),
        trajectory_df['true_z'].max()
    )
    xx, yy = np.meshgrid(
        np.linspace(0, max_range * 1.1, 10),
        np.linspace(0, max_range * 1.1, 10)
    )
    fig.add_trace(go.Surface(
        x=xx, y=yy, z=np.zeros_like(xx),
        colorscale=[[0, 'gray'], [1, 'gray']],
        showscale=False,
        opacity=0.1,
        name='Ground'
    ))

    fig.update_layout(
        title={
            'text': '<b>Extended Kalman Filter - 3D Trajectory Tracking</b><br>' +
                   '<sub>Green: True Path | Blue: Kalman Estimate | Red X: Impact Point</sub>',
            'x': 0.5,
            'xanchor': 'center'
        },
        scene=dict(
            xaxis_title='Range X (m)',
            yaxis_title='Altitude Y (m)',
            zaxis_title='Cross-range Z (m)',
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.2)
            ),
            aspectmode='manual',
            aspectratio=dict(x=1, y=1, z=0.5)
        ),
        height=700,
        hovermode='closest'
    )

    return fig


def create_metrics_plots(trajectory_df, measurements_df, errors_df):
    """Create error analysis and metrics plots."""
    fig = make_subplots(
        rows=3, cols=2,
        subplot_titles=(
            'Position Error vs Time',
            'Velocity Error vs Time',
            'Position Uncertainty (3σ)',
            'Speed Comparison',
            'Radar Range Measurements',
            'Radar Angle Measurements'
        ),
        specs=[
            [{"type": "scatter"}, {"type": "scatter"}],
            [{"type": "scatter"}, {"type": "scatter"}],
            [{"type": "scatter"}, {"type": "scatter"}]
        ],
        vertical_spacing=0.12,
        horizontal_spacing=0.10
    )

    # Calculate position error
    pos_error = np.sqrt(
        (trajectory_df['true_x'] - trajectory_df['est_x'])**2 +
        (trajectory_df['true_y'] - trajectory_df['est_y'])**2 +
        (trajectory_df['true_z'] - trajectory_df['est_z'])**2
    )

    # 1. Position Error vs Time
    fig.add_trace(go.Scatter(
        x=trajectory_df['time'],
        y=pos_error,
        mode='lines',
        name='Position Error',
        line=dict(color='red', width=2)
    ), row=1, col=1)

    # Add 3σ uncertainty band
    fig.add_trace(go.Scatter(
        x=trajectory_df['time'],
        y=trajectory_df['uncertainty'],
        mode='lines',
        name='3σ Uncertainty',
        line=dict(color='blue', width=1, dash='dash'),
        fill=None
    ), row=1, col=1)

    # 2. Velocity Error vs Time
    fig.add_trace(go.Scatter(
        x=errors_df['time'],
        y=errors_df['vel_error'],
        mode='lines',
        name='Velocity Error',
        line=dict(color='orange', width=2)
    ), row=1, col=2)

    # 3. Position Uncertainty (3σ)
    fig.add_trace(go.Scatter(
        x=trajectory_df['time'],
        y=trajectory_df['uncertainty'],
        mode='lines',
        name='Uncertainty',
        line=dict(color='blue', width=2),
        fill='tozeroy'
    ), row=2, col=1)

    # 4. Speed Comparison
    fig.add_trace(go.Scatter(
        x=trajectory_df['time'],
        y=errors_df['true_v'],
        mode='lines',
        name='True Speed',
        line=dict(color='green', width=2)
    ), row=2, col=2)

    fig.add_trace(go.Scatter(
        x=trajectory_df['time'],
        y=errors_df['est_v'],
        mode='lines',
        name='Estimated Speed',
        line=dict(color='blue', width=2, dash='dash')
    ), row=2, col=2)

    # 5. Radar Range Measurements
    fig.add_trace(go.Scatter(
        x=measurements_df['time'],
        y=measurements_df['range'],
        mode='lines',
        name='Range',
        line=dict(color='purple', width=1),
        opacity=0.7
    ), row=3, col=1)

    # True range (from origin)
    true_range = np.sqrt(
        trajectory_df['true_x']**2 +
        trajectory_df['true_y']**2 +
        trajectory_df['true_z']**2
    )
    fig.add_trace(go.Scatter(
        x=trajectory_df['time'],
        y=true_range,
        mode='lines',
        name='True Range',
        line=dict(color='green', width=2)
    ), row=3, col=1)

    # 6. Radar Angle Measurements
    fig.add_trace(go.Scatter(
        x=measurements_df['time'],
        y=np.degrees(measurements_df['azimuth']),
        mode='lines',
        name='Azimuth',
        line=dict(color='cyan', width=1)
    ), row=3, col=2)

    fig.add_trace(go.Scatter(
        x=measurements_df['time'],
        y=np.degrees(measurements_df['elevation']),
        mode='lines',
        name='Elevation',
        line=dict(color='magenta', width=1)
    ), row=3, col=2)

    # Update axes labels
    fig.update_xaxes(title_text='Time (s)', row=1, col=1)
    fig.update_xaxes(title_text='Time (s)', row=1, col=2)
    fig.update_xaxes(title_text='Time (s)', row=2, col=1)
    fig.update_xaxes(title_text='Time (s)', row=2, col=2)
    fig.update_xaxes(title_text='Time (s)', row=3, col=1)
    fig.update_xaxes(title_text='Time (s)', row=3, col=2)

    fig.update_yaxes(title_text='Error (m)', row=1, col=1)
    fig.update_yaxes(title_text='Error (m/s)', row=1, col=2)
    fig.update_yaxes(title_text='Uncertainty (m)', row=2, col=1)
    fig.update_yaxes(title_text='Speed (m/s)', row=2, col=2)
    fig.update_yaxes(title_text='Range (m)', row=3, col=1)
    fig.update_yaxes(title_text='Angle (degrees)', row=3, col=2)

    fig.update_layout(
        title_text='<b>Extended Kalman Filter - Performance Metrics</b>',
        height=900,
        showlegend=False,
        hovermode='x unified'
    )

    return fig


def create_2d_trajectory_comparison(trajectory_df):
    """Create 2D trajectory comparison (top view and side view)."""
    fig = make_subplots(
        rows=1, cols=2,
        subplot_titles=('Top View (X-Z Plane)', 'Side View (X-Y Plane)'),
        specs=[[{"type": "scatter"}, {"type": "scatter"}]]
    )

    # Top view (X-Z)
    fig.add_trace(go.Scatter(
        x=trajectory_df['true_x'],
        y=trajectory_df['true_z'],
        mode='lines',
        name='True Trajectory',
        line=dict(color='green', width=3)
    ), row=1, col=1)

    fig.add_trace(go.Scatter(
        x=trajectory_df['est_x'],
        y=trajectory_df['est_z'],
        mode='lines',
        name='Kalman Estimate',
        line=dict(color='blue', width=2, dash='dash')
    ), row=1, col=1)

    # Markers every 50 points
    for i in range(0, len(trajectory_df), 50):
        fig.add_trace(go.Scatter(
            x=[trajectory_df['est_x'].iloc[i]],
            y=[trajectory_df['est_z'].iloc[i]],
            mode='markers',
            marker=dict(size=6, color='blue'),
            showlegend=False,
            hovertemplate=f't={trajectory_df["time"].iloc[i]:.1f}s<br>X=%{{x:.0f}}m<br>Z=%{{y:.0f}}m'
        ), row=1, col=1)

    # Side view (X-Y)
    fig.add_trace(go.Scatter(
        x=trajectory_df['true_x'],
        y=trajectory_df['true_y'],
        mode='lines',
        name='True Trajectory',
        line=dict(color='green', width=3),
        showlegend=False
    ), row=1, col=2)

    fig.add_trace(go.Scatter(
        x=trajectory_df['est_x'],
        y=trajectory_df['est_y'],
        mode='lines',
        name='Kalman Estimate',
        line=dict(color='blue', width=2, dash='dash'),
        showlegend=False
    ), row=1, col=2)

    # Ground line
    fig.add_hline(y=0, line_dash="dot", line_color="gray",
                 annotation_text="Ground", row=1, col=2)

    # Update axes
    fig.update_xaxes(title_text='Range X (m)', row=1, col=1)
    fig.update_yaxes(title_text='Cross-range Z (m)', row=1, col=1)
    fig.update_xaxes(title_text='Range X (m)', row=1, col=2)
    fig.update_yaxes(title_text='Altitude Y (m)', row=1, col=2)

    fig.update_layout(
        title_text='<b>Extended Kalman Filter - 2D Trajectory Comparison</b>',
        height=500,
        hovermode='closest'
    )

    return fig


def print_statistics(trajectory_df, errors_df):
    """Print tracking statistics."""
    print("\n" + "="*70)
    print("KALMAN FILTER TRACKING STATISTICS")
    print("="*70)

    # Position error statistics
    pos_error = np.sqrt(
        (trajectory_df['true_x'] - trajectory_df['est_x'])**2 +
        (trajectory_df['true_y'] - trajectory_df['est_y'])**2 +
        (trajectory_df['true_z'] - trajectory_df['est_z'])**2
    )

    print("\nPosition Error:")
    print(f"  Mean:   {pos_error.mean():.3f} m")
    print(f"  STD:    {pos_error.std():.3f} m")
    print(f"  RMS:    {np.sqrt(np.mean(pos_error**2)):.3f} m")
    print(f"  Max:    {pos_error.max():.3f} m")
    print(f"  Final:  {pos_error.iloc[-1]:.3f} m")

    # Velocity error statistics
    print("\nVelocity Error:")
    print(f"  Mean:   {errors_df['vel_error'].mean():.3f} m/s")
    print(f"  STD:    {errors_df['vel_error'].std():.3f} m/s")
    print(f"  Max:    {errors_df['vel_error'].max():.3f} m/s")

    # Uncertainty statistics
    print("\nPosition Uncertainty (3-sigma):")
    print(f"  Initial: {trajectory_df['uncertainty'].iloc[0]:.3f} m")
    print(f"  Final:   {trajectory_df['uncertainty'].iloc[-1]:.3f} m")
    print(f"  Min:     {trajectory_df['uncertainty'].min():.3f} m")
    print(f"  Max:     {trajectory_df['uncertainty'].max():.3f} m")

    # Filter health check
    print("\nFilter Health Check:")
    healthy = (pos_error < trajectory_df['uncertainty']).sum()
    total = len(pos_error)
    pct = 100 * healthy / total
    print(f"  {healthy}/{total} points ({pct:.1f}%) within 3-sigma uncertainty")
    if pct > 95:
        print("  [OK] Filter is HEALTHY")
    elif pct > 80:
        print("  [WARN] Filter performance is MODERATE")
    else:
        print("  [ERROR] Filter may be MISCONFIGURED")

    # Convergence analysis
    print("\nConvergence Analysis:")
    for threshold in [10, 5, 1]:
        converged_step = np.argmax(pos_error < threshold)
        if converged_step > 0:
            t_conv = trajectory_df['time'].iloc[converged_step]
            print(f"  Error < {threshold}m at t = {t_conv:.2f} s (step {converged_step})")
        else:
            print(f"  Error never converged below {threshold}m")

    print("\n" + "="*70)


def main():
    """Main function to run visualization."""
    print("="*70)
    print("KALMAN FILTER VISUALIZATION")
    print("="*70)

    # Check if data files exist
    required_files = ['kalman_trajectory.csv', 'kalman_measurements.csv', 'kalman_errors.csv']
    for f in required_files:
        if not Path(f).exists():
            print(f"\nError: {f} not found!")
            print("Run kalman_visualization_demo.exe first to generate data files.")
            return

    print("\nLoading data...")
    trajectory_df, measurements_df, errors_df = load_data()

    print(f"  Loaded {len(trajectory_df)} data points")
    print(f"  Time range: {trajectory_df['time'].min():.2f} - {trajectory_df['time'].max():.2f} s")

    # Print statistics
    print_statistics(trajectory_df, errors_df)

    # Create plots
    print("\nGenerating visualizations...")

    print("  [1/3] 3D Trajectory Plot...")
    fig_3d = create_3d_trajectory_plot(trajectory_df, measurements_df)
    fig_3d.write_html('kalman_visualization.html')
    print("    Saved: kalman_visualization.html")

    print("  [2/3] Metrics Plot...")
    fig_metrics = create_metrics_plots(trajectory_df, measurements_df, errors_df)
    fig_metrics.write_html('kalman_metrics.html')
    print("    Saved: kalman_metrics.html")

    print("  [3/3] 2D Trajectory Comparison...")
    fig_2d = create_2d_trajectory_comparison(trajectory_df)
    fig_2d.write_html('kalman_2d_comparison.html')
    print("    Saved: kalman_2d_comparison.html")

    print("\n" + "="*70)
    print("VISUALIZATION COMPLETE")
    print("="*70)
    print("\nGenerated HTML files:")
    print("  1. kalman_visualization.html  - Interactive 3D trajectory")
    print("  2. kalman_metrics.html        - Error analysis plots")
    print("  3. kalman_2d_comparison.html  - 2D trajectory comparison")
    print("\nOpen these files in a web browser to view the interactive plots.")
    print("="*70)


if __name__ == '__main__':
    main()
