#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from scipy.spatial.transform import Rotation

def read_tum_trajectory(filepath):
    """Read trajectory from TUM format file"""
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

def analyze_trajectory_orientations():
    """Analyze the principal motion directions of each trajectory"""
    
    # Load trajectories
    gt = read_tum_trajectory("groundtruth_normalized.txt")
    fs = read_tum_trajectory("estimated_trajectory_foundationstereo.txt") 
    orig = read_tum_trajectory("estimated_trajectory_original_normalized.txt")
    
    def analyze_motion_directions(df, name):
        positions = df[['tx', 'ty', 'tz']].values
        
        # Calculate ranges
        ranges = np.ptp(positions, axis=0)  # peak-to-peak (max - min)
        
        # Calculate principal components
        centered = positions - np.mean(positions, axis=0)
        cov_matrix = np.cov(centered.T)
        eigenvals, eigenvecs = np.linalg.eig(cov_matrix)
        
        # Sort by eigenvalue (largest first)
        idx = np.argsort(eigenvals)[::-1]
        eigenvals = eigenvals[idx]
        eigenvecs = eigenvecs[:, idx]
        
        print(f"\n📊 {name} Analysis:")
        print(f"  Coordinate Ranges: X={ranges[0]:.3f}m, Y={ranges[1]:.3f}m, Z={ranges[2]:.3f}m")
        print(f"  Principal Components (eigenvalues): {eigenvals}")
        print(f"  Primary motion direction: {eigenvecs[:, 0]}")
        print(f"  Secondary motion direction: {eigenvecs[:, 1]}")
        
        return ranges, eigenvals, eigenvecs, positions
    
    gt_ranges, gt_evals, gt_evecs, gt_pos = analyze_motion_directions(gt, "Ground Truth")
    fs_ranges, fs_evals, fs_evecs, fs_pos = analyze_motion_directions(fs, "FoundationStereo")
    orig_ranges, orig_evals, orig_evecs, orig_pos = analyze_motion_directions(orig, "Original ORB-SLAM3")
    
    return (gt_ranges, gt_evals, gt_evecs, gt_pos), (fs_ranges, fs_evals, fs_evecs, fs_pos), (orig_ranges, orig_evals, orig_evecs, orig_pos)

def try_coordinate_transformations():
    """Try different coordinate transformations to align trajectories"""
    
    print("🔄 Trying different coordinate transformations...")
    
    # Load trajectories
    gt = read_tum_trajectory("groundtruth_normalized.txt")
    fs = read_tum_trajectory("estimated_trajectory_foundationstereo.txt") 
    
    gt_pos = gt[['tx', 'ty', 'tz']].values
    fs_pos = fs[['tx', 'ty', 'tz']].values
    
    # Try different axis mappings
    transformations = {
        'Original': gt_pos,
        'X→X, Y→Z, Z→Y': gt_pos[:, [0, 2, 1]], 
        'X→Y, Y→X, Z→Z': gt_pos[:, [1, 0, 2]],
        'X→Y, Y→Z, Z→X': gt_pos[:, [1, 2, 0]],
        'X→Z, Y→X, Z→Y': gt_pos[:, [2, 0, 1]],
        'X→Z, Y→Y, Z→X': gt_pos[:, [2, 1, 0]],
        'Y→-Z mapping': np.column_stack([gt_pos[:, 0], -gt_pos[:, 2], gt_pos[:, 1]]),
        'Y→Z mapping': np.column_stack([gt_pos[:, 0], gt_pos[:, 2], gt_pos[:, 1]]),
    }
    
    # Calculate similarity metrics for each transformation
    results = {}
    
    for name, gt_transformed in transformations.items():
        # Calculate ranges
        gt_ranges = np.ptp(gt_transformed, axis=0)
        fs_ranges = np.ptp(fs_pos, axis=0)
        
        # Calculate similarity score (how well ranges match)
        range_ratios = np.minimum(gt_ranges, fs_ranges) / np.maximum(gt_ranges, fs_ranges)
        similarity_score = np.mean(range_ratios)
        
        results[name] = {
            'gt_ranges': gt_ranges,
            'fs_ranges': fs_ranges,
            'similarity_score': similarity_score,
            'transformed_pos': gt_transformed
        }
        
        print(f"\n🔍 {name}:")
        print(f"  GT Ranges: X={gt_ranges[0]:.3f}, Y={gt_ranges[1]:.3f}, Z={gt_ranges[2]:.3f}")
        print(f"  FS Ranges: X={fs_ranges[0]:.3f}, Y={fs_ranges[1]:.3f}, Z={fs_ranges[2]:.3f}")
        print(f"  Similarity Score: {similarity_score:.3f}")
    
    # Find best transformation
    best_transform = max(results.keys(), key=lambda k: results[k]['similarity_score'])
    print(f"\n🏆 Best transformation: {best_transform}")
    print(f"   Similarity score: {results[best_transform]['similarity_score']:.3f}")
    
    return results, best_transform

def create_aligned_comparison(results, best_transform):
    """Create visualization with aligned coordinate frames"""
    
    # Load trajectories
    gt = read_tum_trajectory("groundtruth_normalized.txt")
    fs = read_tum_trajectory("estimated_trajectory_foundationstereo.txt") 
    orig = read_tum_trajectory("estimated_trajectory_original_normalized.txt")
    
    gt_pos_original = gt[['tx', 'ty', 'tz']].values
    gt_pos_aligned = results[best_transform]['transformed_pos']
    fs_pos = fs[['tx', 'ty', 'tz']].values
    orig_pos = orig[['tx', 'ty', 'tz']].values
    
    # Create comparison plots
    fig, axes = plt.subplots(2, 3, figsize=(20, 12))
    fig.suptitle('Coordinate Frame Alignment: Ground Truth vs ORB-SLAM3', fontsize=16)
    
    # Original comparison (unfair)
    ax1 = axes[0, 0]
    ax1.plot(gt_pos_original[:, 0], gt_pos_original[:, 1], 'k-', linewidth=3, label='Ground Truth (Original)', alpha=0.8)
    ax1.plot(fs_pos[:, 0], fs_pos[:, 1], 'r-', linewidth=2, label='FoundationStereo', alpha=0.8)
    ax1.plot(orig_pos[:, 0], orig_pos[:, 1], 'b-', linewidth=2, label='Original ORB-SLAM3', alpha=0.8)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('❌ Original (Misaligned Coordinates)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Aligned comparison (fair)
    ax2 = axes[0, 1]
    ax2.plot(gt_pos_aligned[:, 0], gt_pos_aligned[:, 1], 'k-', linewidth=3, label='Ground Truth (Aligned)', alpha=0.8)
    ax2.plot(fs_pos[:, 0], fs_pos[:, 1], 'r-', linewidth=2, label='FoundationStereo', alpha=0.8)
    ax2.plot(orig_pos[:, 0], orig_pos[:, 1], 'b-', linewidth=2, label='Original ORB-SLAM3', alpha=0.8)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title(f'✅ Aligned ({best_transform})')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    # XZ plane comparison
    ax3 = axes[0, 2]
    ax3.plot(gt_pos_aligned[:, 0], gt_pos_aligned[:, 2], 'k-', linewidth=3, label='Ground Truth (Aligned)', alpha=0.8)
    ax3.plot(fs_pos[:, 0], fs_pos[:, 2], 'r-', linewidth=2, label='FoundationStereo', alpha=0.8)
    ax3.plot(orig_pos[:, 0], orig_pos[:, 2], 'b-', linewidth=2, label='Original ORB-SLAM3', alpha=0.8)
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('XZ Plane (Aligned)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    
    # 3D comparison - original
    ax4 = axes[1, 0]
    ax4 = plt.subplot(2, 3, 4, projection='3d')
    ax4.plot(gt_pos_original[:, 0], gt_pos_original[:, 1], gt_pos_original[:, 2], 'k-', linewidth=3, label='Ground Truth (Original)', alpha=0.8)
    ax4.plot(fs_pos[:, 0], fs_pos[:, 1], fs_pos[:, 2], 'r-', linewidth=2, label='FoundationStereo', alpha=0.8)
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_zlabel('Z (m)')
    ax4.set_title('3D View - Original')
    ax4.legend()
    
    # 3D comparison - aligned
    ax5 = axes[1, 1]
    ax5 = plt.subplot(2, 3, 5, projection='3d')
    ax5.plot(gt_pos_aligned[:, 0], gt_pos_aligned[:, 1], gt_pos_aligned[:, 2], 'k-', linewidth=3, label='Ground Truth (Aligned)', alpha=0.8)
    ax5.plot(fs_pos[:, 0], fs_pos[:, 1], fs_pos[:, 2], 'r-', linewidth=2, label='FoundationStereo', alpha=0.8)
    ax5.plot(orig_pos[:, 0], orig_pos[:, 1], orig_pos[:, 2], 'b-', linewidth=2, label='Original ORB-SLAM3', alpha=0.8)
    ax5.set_xlabel('X (m)')
    ax5.set_ylabel('Y (m)')
    ax5.set_zlabel('Z (m)')
    ax5.set_title('3D View - Aligned')
    ax5.legend()
    
    # Range comparison
    ax6 = axes[1, 2]
    
    # Calculate ranges for comparison
    gt_orig_ranges = np.ptp(gt_pos_original, axis=0)
    gt_aligned_ranges = np.ptp(gt_pos_aligned, axis=0)
    fs_ranges = np.ptp(fs_pos, axis=0)
    orig_ranges = np.ptp(orig_pos, axis=0)
    
    x_pos = np.arange(3)
    width = 0.2
    
    ax6.bar(x_pos - width*1.5, gt_orig_ranges, width, label='GT Original', alpha=0.7, color='gray')
    ax6.bar(x_pos - width*0.5, gt_aligned_ranges, width, label='GT Aligned', alpha=0.7, color='black')
    ax6.bar(x_pos + width*0.5, fs_ranges, width, label='FoundationStereo', alpha=0.7, color='red')
    ax6.bar(x_pos + width*1.5, orig_ranges, width, label='Original ORB-SLAM3', alpha=0.7, color='blue')
    
    ax6.set_xlabel('Coordinate Axis')
    ax6.set_ylabel('Range (m)')
    ax6.set_title('Coordinate Range Comparison')
    ax6.set_xticks(x_pos)
    ax6.set_xticklabels(['X', 'Y', 'Z'])
    ax6.legend()
    ax6.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig('aligned_coordinate_comparison.png', dpi=300, bbox_inches='tight')
    print("✅ Saved aligned_coordinate_comparison.png")
    
    # Save aligned ground truth trajectory
    gt_aligned_df = gt.copy()
    gt_aligned_df['tx'] = gt_pos_aligned[:, 0]
    gt_aligned_df['ty'] = gt_pos_aligned[:, 1] 
    gt_aligned_df['tz'] = gt_pos_aligned[:, 2]
    
    # Save as TUM format
    with open('groundtruth_aligned.txt', 'w') as f:
        for _, row in gt_aligned_df.iterrows():
            f.write(f"{row['timestamp']:.9f} {row['tx']:.6f} {row['ty']:.6f} {row['tz']:.6f} "
                   f"{row['qx']:.6f} {row['qy']:.6f} {row['qz']:.6f} {row['qw']:.6f}\n")
    
    print("✅ Saved groundtruth_aligned.txt for fair evaluation")

def main():
    print("🎯 COORDINATE FRAME ALIGNMENT")
    print("="*50)
    print("Goal: Transform ground truth to match SLAM coordinate orientation")
    print("="*50)
    
    # Analyze trajectory orientations
    gt_analysis, fs_analysis, orig_analysis = analyze_trajectory_orientations()
    
    # Try different transformations
    results, best_transform = try_coordinate_transformations()
    
    # Create aligned visualization
    create_aligned_comparison(results, best_transform)
    
    print(f"\n" + "="*60)
    print("🎯 ALIGNMENT RESULTS")
    print("="*60)
    print(f"✅ Best coordinate transformation: {best_transform}")
    print(f"✅ Generated aligned visualization: aligned_coordinate_comparison.png")
    print(f"✅ Generated aligned ground truth: groundtruth_aligned.txt")
    print("\nNow you can see a fair comparison of trajectory shapes!")

if __name__ == "__main__":
    main() 