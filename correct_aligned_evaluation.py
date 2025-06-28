#!/usr/bin/env python3

import os
import sys
import subprocess
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

def run_command(cmd):
    """Run shell command and return result"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.returncode, result.stdout, result.stderr
    except Exception as e:
        print(f"Error running command: {cmd}")
        return 1, "", str(e)

def read_tum_trajectory(filepath):
    """Read trajectory from TUM format file"""
    try:
        data = []
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    parts = line.split()
                    if len(parts) >= 8:
                        timestamp = float(parts[0])
                        tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                        qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                        data.append([timestamp, tx, ty, tz, qx, qy, qz, qw])
        
        df = pd.DataFrame(data, columns=['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
        return df
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return None

def analyze_coordinate_systems(trajectories):
    """Analyze the coordinate system differences"""
    print("\n🔍 Coordinate System Analysis:")
    print("=" * 60)
    
    for name, df in trajectories.items():
        if df is not None and len(df) > 0:
            start_pos = df.iloc[0][['tx', 'ty', 'tz']].values
            end_pos = df.iloc[-1][['tx', 'ty', 'tz']].values
            mean_pos = df[['tx', 'ty', 'tz']].mean().values
            
            print(f"{name.upper()}:")
            print(f"  Start Position: ({start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f})")
            print(f"  End Position:   ({end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f})")
            print(f"  Mean Position:  ({mean_pos[0]:.3f}, {mean_pos[1]:.3f}, {mean_pos[2]:.3f})")
            print(f"  Poses:          {len(df)}")
            print()

def create_coordinate_comparison_plot(trajectories):
    """Create plots showing the coordinate system differences"""
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Coordinate System Analysis: Ground Truth vs ORB-SLAM3 Methods', fontsize=16)
    
    colors = {'ground_truth': 'black', 'foundationstereo': 'red', 'original': 'blue'}
    
    # Raw trajectories (no alignment) - THE PROBLEM
    ax1 = axes[0, 0]
    for name, df in trajectories.items():
        if name in colors and df is not None:
            ax1.plot(df['tx'], df['ty'], color=colors[name], label=name, 
                    linewidth=3 if name == 'ground_truth' else 1.5, alpha=0.8)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('❌ Raw Trajectories (WRONG - Different Coordinate Systems)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Ground truth only
    ax2 = axes[0, 1]
    if 'ground_truth' in trajectories and trajectories['ground_truth'] is not None:
        df = trajectories['ground_truth']
        ax2.plot(df['tx'], df['ty'], color='black', label='Ground Truth', linewidth=3, alpha=0.8)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('✅ Ground Truth Trajectory (World Coordinates)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
    
    # ORB-SLAM3 methods only (same coordinate system)
    ax3 = axes[1, 0]
    for name, df in trajectories.items():
        if name in colors and name != 'ground_truth' and df is not None:
            ax3.plot(df['tx'], df['ty'], color=colors[name], label=name, linewidth=1.5, alpha=0.8)
    
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.set_title('ORB-SLAM3 Methods (Relative Coordinates from 0,0,0)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    
    # Distance from origin over time
    ax4 = axes[1, 1]
    for name, df in trajectories.items():
        if name in colors and df is not None:
            t_norm = df['timestamp'] - df['timestamp'].min()
            distances = np.sqrt(df['tx']**2 + df['ty']**2 + df['tz']**2)
            ax4.plot(t_norm, distances, color=colors[name], label=name, 
                    linewidth=3 if name == 'ground_truth' else 1.5, alpha=0.8)
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Distance from Origin (m)')
    ax4.set_title('Distance from Origin vs Time')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('coordinate_system_problem_diagnosis.png', dpi=300, bbox_inches='tight')
    print("✓ Saved coordinate_system_problem_diagnosis.png")

def run_aligned_evaluation():
    """Run evo evaluation with proper alignment - THE CORRECT WAY"""
    
    print("\n🔧 Running ALIGNED Trajectory Evaluation (CORRECT METHOD):")
    print("=" * 70)
    
    gt_file = "groundtruth.txt"
    
    methods = {
        'foundationstereo': "estimated_trajectory_foundationstereo.txt",
        'original': "estimated_trajectory_original_normalized.txt"
    }
    
    results = {}
    
    if not os.path.exists(gt_file):
        print(f"❌ Ground truth file not found: {gt_file}")
        return results
    
    for method_name, trajectory_file in methods.items():
        if os.path.exists(trajectory_file):
            print(f"\n📊 Evaluating {method_name.upper()} with ALIGNMENT...")
            
            # APE with Umeyama alignment (translation + rotation + scale)
            print("  🔄 Running APE with Umeyama alignment...")
            cmd = f"evo_ape tum {gt_file} {trajectory_file} --align --correct_scale --t_max_diff 1.0 --plot --save_plot aligned_{method_name}_ape.png --save_results aligned_{method_name}_ape.zip"
            returncode, stdout, stderr = run_command(cmd)
            
            if returncode == 0:
                print(f"  ✅ APE (aligned) evaluation successful")
                # Extract RMSE from output
                for line in stdout.split('\n'):
                    if 'rmse' in line.lower():
                        try:
                            parts = line.split()
                            rmse_val = float(parts[-1])
                            results[f"{method_name}_ape_aligned_rmse"] = rmse_val
                            print(f"    📍 ALIGNED APE RMSE: {rmse_val:.6f} m")
                            break
                        except:
                            pass
            else:
                print(f"  ❌ APE evaluation failed: {stderr}")
            
            # RPE with alignment
            print("  🔄 Running RPE with alignment...")
            cmd = f"evo_rpe tum {gt_file} {trajectory_file} --align --correct_scale --t_max_diff 1.0 --plot --save_plot aligned_{method_name}_rpe.png --save_results aligned_{method_name}_rpe.zip"
            returncode, stdout, stderr = run_command(cmd)
            
            if returncode == 0:
                print(f"  ✅ RPE (aligned) evaluation successful")
                # Extract RMSE from output
                for line in stdout.split('\n'):
                    if 'rmse' in line.lower():
                        try:
                            parts = line.split()
                            rmse_val = float(parts[-1])
                            results[f"{method_name}_rpe_aligned_rmse"] = rmse_val
                            print(f"    📏 ALIGNED RPE RMSE: {rmse_val:.6f} m")
                            break
                        except:
                            pass
            else:
                print(f"  ❌ RPE evaluation failed: {stderr}")
            
            # Trajectory comparison with alignment
            print("  🔄 Generating aligned trajectory plot...")
            cmd = f"evo_traj tum {gt_file} {trajectory_file} --ref {gt_file} --align --correct_scale --plot --save_plot aligned_{method_name}_trajectory.png"
            returncode, stdout, stderr = run_command(cmd)
            
            if returncode == 0:
                print(f"  ✅ Aligned trajectory plot saved")
            else:
                print(f"  ⚠️  Trajectory plot failed: {stderr}")
        else:
            print(f"❌ Trajectory file not found: {trajectory_file}")
    
    return results

def main():
    print("🎯 CORRECT ORB-SLAM3 vs Ground Truth Evaluation")
    print("=" * 60)
    print("Problem: Raw comparison uses different coordinate systems!")
    print("Solution: Use Umeyama alignment for proper evaluation")
    print("=" * 60)
    
    # Load trajectories for coordinate system analysis
    trajectories = {}
    
    if os.path.exists("groundtruth.txt"):
        trajectories['ground_truth'] = read_tum_trajectory("groundtruth.txt")
    
    if os.path.exists("estimated_trajectory_foundationstereo.txt"):
        trajectories['foundationstereo'] = read_tum_trajectory("estimated_trajectory_foundationstereo.txt")
        
    if os.path.exists("estimated_trajectory_original_normalized.txt"):
        trajectories['original'] = read_tum_trajectory("estimated_trajectory_original_normalized.txt")
    
    # Analyze coordinate system differences
    analyze_coordinate_systems(trajectories)
    
    # Create diagnostic plots
    create_coordinate_comparison_plot(trajectories)
    
    # Run proper aligned evaluation
    results = run_aligned_evaluation()
    
    # Summary
    print("\n" + "=" * 70)
    print("🏆 ALIGNED EVALUATION RESULTS SUMMARY")
    print("=" * 70)
    
    if results:
        for method in ['foundationstereo', 'original']:
            ape_key = f"{method}_ape_aligned_rmse"
            rpe_key = f"{method}_rpe_aligned_rmse"
            
            if ape_key in results and rpe_key in results:
                print(f"\n🔬 {method.upper()}:")
                print(f"  📍 APE RMSE (aligned): {results[ape_key]:.6f} m")
                print(f"  📏 RPE RMSE (aligned): {results[rpe_key]:.6f} m")
    
    print("\n" + "=" * 70)
    print("✅ This is the CORRECT evaluation methodology!")
    print("✅ Alignment accounts for coordinate system differences")
    print("✅ Results now show true SLAM accuracy vs ground truth")
    print("=" * 70)

if __name__ == "__main__":
    main() 