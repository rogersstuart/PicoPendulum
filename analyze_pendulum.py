#!/usr/bin/env python3
"""
Pendulum Control Analysis Script
Analyzes CSV data from the pendulum simulator
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import sys
from pathlib import Path

def load_data(filename):
    """Load CSV data from simulator"""
    try:
        data = pd.read_csv(filename)
        print(f"Loaded {len(data)} data points from {filename}")
        return data
    except FileNotFoundError:
        print(f"Error: File {filename} not found")
        return None
    except Exception as e:
        print(f"Error loading data: {e}")
        return None

def analyze_performance(data):
    """Analyze control performance metrics"""
    print("\n=== Performance Analysis ===")
    
    # Basic statistics
    final_angle = data['angle_deg'].iloc[-1]
    max_angle = data['angle_deg'].abs().max()
    rms_angle = np.sqrt(np.mean(data['angle_deg']**2))
    
    print(f"Final angle: {final_angle:.2f}° from upright")
    print(f"Maximum angle: {max_angle:.2f}°")
    print(f"RMS angle error: {rms_angle:.2f}°")
    
    # Energy analysis
    final_energy = data['energy'].iloc[-1]
    target_energy = 0.2 * 9.81 * (0.3048/2) * 2.2  # Approximate target
    energy_error = abs(final_energy - target_energy)
    
    print(f"Final energy: {final_energy:.4f}J")
    print(f"Target energy: {target_energy:.4f}J")
    print(f"Energy error: {energy_error:.4f}J")
    
    # Control effort
    max_motor = data['motor_cmd'].abs().max()
    rms_motor = np.sqrt(np.mean(data['motor_cmd']**2))
    
    print(f"Maximum motor command: {max_motor:.3f}")
    print(f"RMS motor effort: {rms_motor:.3f}")
    
    # State analysis
    states = data['state'].value_counts().sort_index()
    state_names = {0: 'IDLE', 1: 'CALIB', 2: 'SWINGUP', 3: 'CATCH', 4: 'BALANCE', 5: 'FAULT'}
    
    print(f"\nState distribution:")
    for state, count in states.items():
        percentage = (count / len(data)) * 100
        state_name = state_names.get(state, f'UNKNOWN_{state}')
        print(f"  {state_name}: {count} samples ({percentage:.1f}%)")
    
    # Success metrics
    balance_time = len(data[data['state'] == 4]) * 0.01  # 10ms per sample
    upright_time = len(data[data['angle_deg'].abs() < 5]) * 0.01
    
    print(f"\nBalance mode time: {balance_time:.2f}s")
    print(f"Time within ±5°: {upright_time:.2f}s")
    
    # Performance rating
    if final_angle < 5 and balance_time > 2:
        print("✓ EXCELLENT: Stable upright balance achieved")
    elif final_angle < 15 and balance_time > 1:
        print("✓ GOOD: Nearly balanced, minor tuning needed")
    elif max_angle < 90 and final_energy > target_energy * 0.5:
        print("○ FAIR: Swing-up working, balance needs improvement")
    else:
        print("✗ POOR: Significant tuning required")

def plot_results(data, save_plots=False):
    """Create comprehensive plots of simulation results"""
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Pendulum Control Simulation Results', fontsize=16)
    
    time = data['time']
    
    # Plot 1: Angle vs Time
    ax1 = axes[0, 0]
    ax1.plot(time, data['angle_deg'], 'b-', linewidth=1.5, label='Angle from upright')
    ax1.axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Upright target')
    ax1.axhline(y=5, color='orange', linestyle=':', alpha=0.7, label='±5° tolerance')
    ax1.axhline(y=-5, color='orange', linestyle=':', alpha=0.7)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (degrees)')
    ax1.set_title('Pendulum Angle')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Plot 2: Motor Command vs Time
    ax2 = axes[0, 1]
    ax2.plot(time, data['motor_cmd'], 'g-', linewidth=1.5, label='Motor command')
    ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax2.axhline(y=1, color='r', linestyle='--', alpha=0.5, label='Saturation limits')
    ax2.axhline(y=-1, color='r', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Motor Command')
    ax2.set_title('Control Effort')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_ylim(-1.2, 1.2)
    
    # Plot 3: Energy vs Time
    ax3 = axes[1, 0]
    target_energy = 0.2 * 9.81 * (0.3048/2) * 2.2
    ax3.plot(time, data['energy'], 'purple', linewidth=1.5, label='Total energy')
    ax3.axhline(y=target_energy, color='r', linestyle='--', alpha=0.7, label='Target energy')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Energy (J)')
    ax3.set_title('System Energy')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # Plot 4: Control State vs Time
    ax4 = axes[1, 1]
    state_colors = {0: 'gray', 1: 'yellow', 2: 'blue', 3: 'orange', 4: 'green', 5: 'red'}
    state_names = {0: 'IDLE', 1: 'CALIB', 2: 'SWINGUP', 3: 'CATCH', 4: 'BALANCE', 5: 'FAULT'}
    
    # Plot states as colored regions
    current_state = data['state'].iloc[0]
    start_time = time.iloc[0]
    
    for i in range(1, len(data)):
        if data['state'].iloc[i] != current_state or i == len(data) - 1:
            end_time = time.iloc[i-1] if i < len(data) - 1 else time.iloc[i]
            color = state_colors.get(current_state, 'black')
            label = state_names.get(current_state, f'STATE_{current_state}')
            ax4.axvspan(start_time, end_time, alpha=0.7, color=color, label=label)
            current_state = data['state'].iloc[i]
            start_time = time.iloc[i]
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Control State')
    ax4.set_title('State Machine')
    ax4.set_ylim(-0.5, 5.5)
    
    # Remove duplicate labels
    handles, labels = ax4.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax4.legend(by_label.values(), by_label.keys(), loc='upper right')
    
    plt.tight_layout()
    
    if save_plots:
        plt.savefig('pendulum_analysis.png', dpi=300, bbox_inches='tight')
        print("Plots saved to pendulum_analysis.png")
    
    plt.show()

def plot_phase_portrait(data, save_plots=False):
    """Create phase portrait (angle vs angular velocity)"""
    
    # Calculate approximate angular velocity from angle difference
    angles_rad = data['angle_deg'] * np.pi / 180
    dt = data['time'].iloc[1] - data['time'].iloc[0]
    omega_approx = np.gradient(angles_rad) / dt
    
    plt.figure(figsize=(10, 8))
    
    # Color points by time
    scatter = plt.scatter(data['angle_deg'], omega_approx, c=data['time'], 
                         cmap='viridis', alpha=0.6, s=20)
    
    # Mark start and end points
    plt.plot(data['angle_deg'].iloc[0], omega_approx[0], 'go', markersize=10, label='Start')
    plt.plot(data['angle_deg'].iloc[-1], omega_approx[-1], 'ro', markersize=10, label='End')
    
    # Mark equilibrium points
    plt.axvline(x=0, color='r', linestyle='--', alpha=0.5, label='Upright equilibrium')
    plt.axvline(x=180, color='orange', linestyle='--', alpha=0.5, label='Hanging equilibrium')
    plt.axvline(x=-180, color='orange', linestyle='--', alpha=0.5)
    
    plt.xlabel('Angle from Upright (degrees)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Phase Portrait')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.colorbar(scatter, label='Time (s)')
    
    if save_plots:
        plt.savefig('phase_portrait.png', dpi=300, bbox_inches='tight')
        print("Phase portrait saved to phase_portrait.png")
    
    plt.show()

def export_summary(data, filename="analysis_summary.txt"):
    """Export analysis summary to text file"""
    
    with open(filename, 'w') as f:
        f.write("Pendulum Control Simulation Analysis Summary\n")
        f.write("=" * 50 + "\n\n")
        
        # Basic info
        f.write(f"Simulation duration: {data['time'].iloc[-1]:.2f} seconds\n")
        f.write(f"Data points: {len(data)}\n")
        f.write(f"Sample rate: {1/(data['time'].iloc[1] - data['time'].iloc[0]):.0f} Hz\n\n")
        
        # Performance metrics
        final_angle = data['angle_deg'].iloc[-1]
        max_angle = data['angle_deg'].abs().max()
        rms_angle = np.sqrt(np.mean(data['angle_deg']**2))
        
        f.write("Performance Metrics:\n")
        f.write(f"  Final angle error: {final_angle:.2f}°\n")
        f.write(f"  Maximum angle: {max_angle:.2f}°\n")
        f.write(f"  RMS angle error: {rms_angle:.2f}°\n")
        
        max_motor = data['motor_cmd'].abs().max()
        rms_motor = np.sqrt(np.mean(data['motor_cmd']**2))
        f.write(f"  Maximum motor command: {max_motor:.3f}\n")
        f.write(f"  RMS motor effort: {rms_motor:.3f}\n\n")
        
        # State statistics
        states = data['state'].value_counts().sort_index()
        state_names = {0: 'IDLE', 1: 'CALIB', 2: 'SWINGUP', 3: 'CATCH', 4: 'BALANCE', 5: 'FAULT'}
        
        f.write("State Distribution:\n")
        for state, count in states.items():
            percentage = (count / len(data)) * 100
            state_name = state_names.get(state, f'UNKNOWN_{state}')
            f.write(f"  {state_name}: {count} samples ({percentage:.1f}%)\n")
        
        f.write(f"\nAnalysis complete at {pd.Timestamp.now()}\n")
    
    print(f"Summary exported to {filename}")

def main():
    parser = argparse.ArgumentParser(description='Analyze pendulum simulation data')
    parser.add_argument('filename', help='CSV file from simulator')
    parser.add_argument('--no-plots', action='store_true', help='Skip plotting')
    parser.add_argument('--save-plots', action='store_true', help='Save plots to files')
    parser.add_argument('--phase-portrait', action='store_true', help='Show phase portrait')
    parser.add_argument('--export-summary', action='store_true', help='Export analysis summary')
    
    args = parser.parse_args()
    
    # Check if file exists
    if not Path(args.filename).exists():
        print(f"Error: File '{args.filename}' not found")
        sys.exit(1)
    
    # Load and analyze data
    data = load_data(args.filename)
    if data is None:
        sys.exit(1)
    
    # Perform analysis
    analyze_performance(data)
    
    # Create plots
    if not args.no_plots:
        plot_results(data, args.save_plots)
        
        if args.phase_portrait:
            plot_phase_portrait(data, args.save_plots)
    
    # Export summary
    if args.export_summary:
        export_summary(data)
    
    print("\nAnalysis complete!")

if __name__ == "__main__":
    main()