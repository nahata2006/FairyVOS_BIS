#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import argparse
import os
import sys

def read_tum_trajectory(filepath, max_poses=None):
    """Read trajectory from TUM format file"""
    if not os.path.exists(filepath):
        print(f"❌ Error: File {filepath} not found!")
        return None
        
    data = []
    count = 0
    try:
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
                        count += 1
                        if max_poses and count >= max_poses:
                            break
        
        if not data:
            print(f"❌ Error: No valid trajectory data found in {filepath}")
            return None
            
        df = pd.DataFrame(data, columns=['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
        print(f"✅ Loaded {len(df)} poses from {filepath}")
        return df
        
    except Exception as e:
        print(f"❌ Error reading {filepath}: {e}")
        return None

def create_slam_trajectory_comparison(slam_file, gt_file, output_name=None, slam_name="SLAM"):
    """Create comprehensive trajectory visualization comparing SLAM output with ground truth"""
    
    print(f"🎯 SLAM TRAJECTORY VISUALIZATION")
    print("="*50)
    print(f"SLAM File: {slam_file}")
    print(f"Ground Truth File: {gt_file}")
    print("="*50)
    
    # Load trajectories
    print(f"\n📁 Loading trajectory data...")
    
    # Load SLAM trajectory
    slam_df = read_tum_trajectory(slam_file)
    if slam_df is None:
        print("❌ Failed to load SLAM trajectory")
        return False
    
    # Load ground truth
    gt_df = read_tum_trajectory(gt_file)
    if gt_df is None:
        print("❌ Failed to load ground truth")
        return False
    
    print(f"\n✅ Loaded trajectories:")
    print(f"  Ground Truth: {len(gt_df)} poses")
    print(f"  {slam_name}: {len(slam_df)} poses") 
    
    # Calculate trajectory statistics
    def calc_stats(df, name):
        if len(df) < 2:
            return 0, 0
        path_length = np.sum(np.sqrt(np.diff(df['tx'])**2 + np.diff(df['ty'])**2 + np.diff(df['tz'])**2))
        duration = df['timestamp'].max() - df['timestamp'].min()
        range_x = df['tx'].max() - df['tx'].min()
        range_y = df['ty'].max() - df['ty'].min()  
        range_z = df['tz'].max() - df['tz'].min()
        print(f"\n📊 {name}:")
        print(f"  Path Length: {path_length:.3f} m")
        print(f"  Duration: {duration:.1f} s")
        print(f"  X Range: {range_x:.3f} m")
        print(f"  Y Range: {range_y:.3f} m")
        print(f"  Z Range: {range_z:.3f} m")
        return path_length, duration
    
    print("\n" + "="*50)
    print("📊 TRAJECTORY STATISTICS")
    print("="*50)
    
    gt_path, gt_dur = calc_stats(gt_df, "Ground Truth")
    slam_path, slam_dur = calc_stats(slam_df, slam_name)
    
    # Create comprehensive visualization
    fig = plt.figure(figsize=(16, 12))
    
    # 2D XY trajectory plot
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(gt_df['tx'], gt_df['ty'], 'k-', linewidth=3, label=f'Ground Truth ({len(gt_df)} poses)', alpha=0.8)
    ax1.plot(slam_df['tx'], slam_df['ty'], 'r-', linewidth=2, label=f'{slam_name} ({len(slam_df)} poses)', alpha=0.8)
    
    # Mark start points
    ax1.plot(gt_df['tx'].iloc[0], gt_df['ty'].iloc[0], 'ko', markersize=8, label='GT Start')
    ax1.plot(slam_df['tx'].iloc[0], slam_df['ty'].iloc[0], 'ro', markersize=8, label='SLAM Start')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title(f'XY Trajectory Comparison')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 2D XZ trajectory plot  
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(gt_df['tx'], gt_df['tz'], 'k-', linewidth=3, label='Ground Truth', alpha=0.8)
    ax2.plot(slam_df['tx'], slam_df['tz'], 'r-', linewidth=2, label=slam_name, alpha=0.8)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Z (m)')
    ax2.set_title('XZ Plane')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    # Position vs Time plots
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(gt_df['timestamp'], gt_df['tx'], 'k-', linewidth=2, label='GT X', alpha=0.7)
    ax3.plot(gt_df['timestamp'], gt_df['ty'], 'k--', linewidth=2, label='GT Y', alpha=0.7)
    ax3.plot(slam_df['timestamp'], slam_df['tx'], 'r-', linewidth=1.5, label='SLAM X', alpha=0.8)
    ax3.plot(slam_df['timestamp'], slam_df['ty'], 'r--', linewidth=1.5, label='SLAM Y', alpha=0.8)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (m)')
    ax3.set_title('Position vs Time')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 3D trajectory plot
    ax4 = plt.subplot(2, 3, 4, projection='3d')
    ax4.plot(gt_df['tx'], gt_df['ty'], gt_df['tz'], 'k-', linewidth=3, label='Ground Truth', alpha=0.8)
    ax4.plot(slam_df['tx'], slam_df['ty'], slam_df['tz'], 'r-', linewidth=2, label=slam_name, alpha=0.8)
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_zlabel('Z (m)')
    ax4.set_title('3D Trajectory')
    ax4.legend()
    
    # Coordinate range comparison
    ax5 = plt.subplot(2, 3, 5)
    
    gt_ranges = np.array([gt_df['tx'].max() - gt_df['tx'].min(), 
                         gt_df['ty'].max() - gt_df['ty'].min(), 
                         gt_df['tz'].max() - gt_df['tz'].min()])
    slam_ranges = np.array([slam_df['tx'].max() - slam_df['tx'].min(), 
                           slam_df['ty'].max() - slam_df['ty'].min(), 
                           slam_df['tz'].max() - slam_df['tz'].min()])
    
    x_pos = np.arange(3)
    width = 0.35
    
    ax5.bar(x_pos - width/2, gt_ranges, width, label='Ground Truth', alpha=0.8, color='black')
    ax5.bar(x_pos + width/2, slam_ranges, width, label=slam_name, alpha=0.8, color='red')
    
    ax5.set_xlabel('Coordinate Axis')
    ax5.set_ylabel('Range (m)')
    ax5.set_title('Coordinate Range Comparison')
    ax5.set_xticks(x_pos)
    ax5.set_xticklabels(['X', 'Y', 'Z'])
    ax5.legend()
    ax5.grid(True, alpha=0.3, axis='y')
    
    # Add value labels
    for i, (gt_r, slam_r) in enumerate(zip(gt_ranges, slam_ranges)):
        ax5.text(i - width/2, gt_r + 0.1, f'{gt_r:.1f}', ha='center', va='bottom', fontsize=8)
        ax5.text(i + width/2, slam_r + 0.1, f'{slam_r:.1f}', ha='center', va='bottom', fontsize=8)
    
    # Coverage comparison
    ax6 = plt.subplot(2, 3, 6)
    
    coverage_metrics = ['Path Length\n(m)', 'Duration\n(s)', 'Pose Count']
    gt_values = [gt_path, gt_dur, len(gt_df)]
    slam_values = [slam_path, slam_dur, len(slam_df)]
    
    x_pos = np.arange(len(coverage_metrics))
    width = 0.35
    
    ax6.bar(x_pos - width/2, gt_values, width, label='Ground Truth', alpha=0.8, color='black')
    ax6.bar(x_pos + width/2, slam_values, width, label=slam_name, alpha=0.8, color='red')
    
    ax6.set_xlabel('Metric')
    ax6.set_ylabel('Value')
    ax6.set_title('Coverage Comparison')
    ax6.set_xticks(x_pos)
    ax6.set_xticklabels(coverage_metrics)
    ax6.legend()
    ax6.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    
    # Save with appropriate name
    if output_name is None:
        output_name = f"{slam_name.lower().replace(' ', '_')}_trajectory_comparison.png"
    
    plt.savefig(output_name, dpi=300, bbox_inches='tight')
    print(f"\n✅ Saved {output_name}")
    
    # Summary
    print(f"\n" + "="*60)
    print(f"🎯 {slam_name.upper()} TRAJECTORY COMPARISON SUMMARY")
    print("="*60)
    if gt_path > 0:
        print(f"📏 Path Length Ratio: {slam_path/gt_path*100:.1f}% of ground truth")
    if gt_dur > 0:
        print(f"⏱️ Duration Coverage: {slam_dur/gt_dur*100:.1f}% of ground truth")
    print(f"📊 Pose Count Ratio: {len(slam_df)/len(gt_df)*100:.1f}% of ground truth")
    
    print(f"\n✅ Trajectory comparison complete!")
    return True

def main():
    parser = argparse.ArgumentParser(description='Visualize SLAM trajectory compared to ground truth')
    parser.add_argument('slam_file', help='SLAM trajectory file (TUM format)')
    parser.add_argument('groundtruth_file', help='Ground truth file (TUM format)')
    parser.add_argument('--slam-name', default='SLAM', help='Name for SLAM method (default: SLAM)')
    parser.add_argument('--output', '-o', help='Output image filename (default: auto-generated)')
    parser.add_argument('--max-poses', type=int, help='Maximum number of poses to load')
    
    args = parser.parse_args()
    
    # Check if files exist
    if not os.path.exists(args.slam_file):
        print(f"❌ Error: SLAM file '{args.slam_file}' not found!")
        sys.exit(1)
        
    if not os.path.exists(args.groundtruth_file):
        print(f"❌ Error: Ground truth file '{args.groundtruth_file}' not found!")
        sys.exit(1)
    
    # Create visualization
    success = create_slam_trajectory_comparison(
        args.slam_file, 
        args.groundtruth_file, 
        args.output, 
        args.slam_name
    )
    
    if success:
        print(f"\n🎉 Success! Check the generated visualization image.")
    else:
        print(f"\n❌ Failed to create trajectory comparison")
        sys.exit(1)

if __name__ == "__main__":
    main() 