#!/usr/bin/env python3
"""
Simple Path Comparison Visualization
Shows waypoints vs smoothed cubic spline path
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


def plot_path_comparison(waypoints, save_path='path_comparison.png'):
    """Generate beautiful path comparison plot"""
    
    # Convert waypoints to array
    waypoints = np.array(waypoints)
    
    # Generate smooth path using cubic splines
    distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1)))
    distances = np.insert(distances, 0, 0)
    
    cs_x = CubicSpline(distances, waypoints[:, 0], bc_type='natural')
    cs_y = CubicSpline(distances, waypoints[:, 1], bc_type='natural')
    
    alpha = np.linspace(0, distances[-1], 200)
    smooth_x = cs_x(alpha)
    smooth_y = cs_y(alpha)
    
    # Create figure
    plt.figure(figsize=(12, 8))
    
    # Plot discrete waypoints
    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-',
             label='Discrete Waypoints', markersize=12, linewidth=2.5,
             markeredgecolor='darkred', markeredgewidth=1.5)
    
    # Plot smooth cubic spline path
    plt.plot(smooth_x, smooth_y, 'b-',
             label='Cubic Spline Smoothed Path', linewidth=3, alpha=0.8)
    
    # Mark start and end
    plt.plot(waypoints[0, 0], waypoints[0, 1], 'go',
             markersize=15, label='Start', markeredgecolor='darkgreen',
             markeredgewidth=2)
    plt.plot(waypoints[-1, 0], waypoints[-1, 1], 'ms',
             markersize=15, label='Goal', markeredgecolor='purple',
             markeredgewidth=2)
    
    # Add waypoint labels
    for i, (x, y) in enumerate(waypoints):
        plt.annotate(f'WP{i}', (x, y), textcoords="offset points",
                    xytext=(0, 10), ha='center', fontsize=10,
                    fontweight='bold', color='darkred')
    
    # Styling
    plt.xlabel('X Position (meters)', fontsize=14, fontweight='bold')
    plt.ylabel('Y Position (meters)', fontsize=14, fontweight='bold')
    plt.title('Path Smoothing: Discrete Waypoints → Cubic Spline',
              fontsize=16, fontweight='bold', pad=20)
    plt.legend(fontsize=12, loc='best', framealpha=0.9)
    plt.grid(True, alpha=0.3, linestyle='--')
    plt.axis('equal')
    plt.tight_layout()
    
    # Save
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"✅ Saved plot: {save_path}")
    
    return plt


def plot_curvature_analysis(waypoints, save_path='curvature_analysis.png'):
    """Show why cubic splines are better - curvature comparison"""
    
    waypoints = np.array(waypoints)
    
    # Cubic spline
    distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1)))
    distances = np.insert(distances, 0, 0)
    cs_x = CubicSpline(distances, waypoints[:, 0], bc_type='natural')
    cs_y = CubicSpline(distances, waypoints[:, 1], bc_type='natural')
    alpha = np.linspace(0, distances[-1], 200)
    smooth_x = cs_x(alpha)
    smooth_y = cs_y(alpha)
    
    # Linear interpolation
    linear_x = np.interp(alpha, distances, waypoints[:, 0])
    linear_y = np.interp(alpha, distances, waypoints[:, 1])
    
    # Create comparison
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # Left: Path comparison
    ax1.plot(waypoints[:, 0], waypoints[:, 1], 'ko',
            markersize=10, label='Waypoints')
    ax1.plot(linear_x, linear_y, 'r--',
            linewidth=2, label='Linear (Sharp Corners)', alpha=0.7)
    ax1.plot(smooth_x, smooth_y, 'b-',
            linewidth=3, label='Cubic Spline (Smooth)', alpha=0.8)
    ax1.set_xlabel('X Position (m)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Y Position (m)', fontsize=12, fontweight='bold')
    ax1.set_title('Path Comparison', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Right: Smoothness comparison
    ax2.text(0.5, 0.7, 'Cubic Spline Properties:', ha='center',
            fontsize=14, fontweight='bold', transform=ax2.transAxes)
    
    properties = [
        '✓ C² Continuity (Smooth Acceleration)',
        '✓ No Sharp Corners',
        '✓ Passes Through All Waypoints',
        '✓ Minimal Curvature',
        '✓ Comfortable for Robot Motion',
        '',
        'Linear Interpolation:',
        '✗ Sharp Direction Changes',
        '✗ Discontinuous Acceleration',
        '✗ Jerky Robot Motion'
    ]
    
    y_pos = 0.55
    for prop in properties:
        color = 'green' if '✓' in prop else ('red' if '✗' in prop else 'black')
        weight = 'bold' if 'Linear' in prop or 'Cubic' in prop else 'normal'
        ax2.text(0.5, y_pos, prop, ha='center', fontsize=11,
                transform=ax2.transAxes, color=color, fontweight=weight)
        y_pos -= 0.06
    
    ax2.axis('off')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"✅ Saved plot: {save_path}")
    
    return fig


def generate_demo_plots():
    """Generate demo plots with sample waypoints"""
    
    # Sample waypoints (customize these!)
    waypoints = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 1.0),
        (3.0, 1.0),
        (4.0, 2.0),
        (4.0, 3.0)
    ]
    
    print("\n" + "="*60)
    print("📊 GENERATING PATH VISUALIZATION PLOTS")
    print("="*60 + "\n")
    
    # Generate plots
    plot_path_comparison(waypoints, 'path_comparison.png')
    plot_curvature_analysis(waypoints, 'curvature_analysis.png')
    
    print("\n" + "="*60)
    print("✅ ALL PLOTS GENERATED SUCCESSFULLY!")
    print("="*60)
    print("\nFiles created:")
    print("  📄 path_comparison.png - Main path smoothing visualization")
    print("  📄 curvature_analysis.png - Why cubic splines are better")
    print("\n💡 Use these plots in your documentation and video!\n")
    
    # Show plots
    plt.show()


if __name__ == '__main__':
    generate_demo_plots()