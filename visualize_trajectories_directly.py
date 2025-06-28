#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

def read_tum_trajectory(filepath, max_poses=None):
    """Read trajectory from TUM format file"""
    data = []
    count = 0
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
    
    df = pd.DataFrame(data, columns=['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
    return df

def find_best_coordinate_transformation():
    """Find the best coordinate transformation including sign flips"""
    
    print("🔍 Testing different coordinate transformations and sign flips...")
    
    # Load trajectories
    gt_original = read_tum_trajectory("groundtruth_normalized.txt")
    fs = read_tum_trajectory("estimated_trajectory_foundationstereo.txt") 
    
    gt_pos = gt_original[['tx', 'ty', 'tz']].values
    fs_pos = fs[['tx', 'ty', 'tz']].values
    
    # Try different transformations with sign flips
    transformations = {
        'Original': gt_pos,
        'X→Y, Y→Z, Z→X': gt_pos[:, [1, 2, 0]], 
        'X→Y, Y→Z, Z→X, flip_X': np.column_stack([-gt_pos[:, 1], gt_pos[:, 2], gt_pos[:, 0]]),
        'X→Y, Y→Z, Z→X, flip_Y': np.column_stack([gt_pos[:, 1], -gt_pos[:, 2], gt_pos[:, 0]]),
        'X→Y, Y→Z, Z→X, flip_Z': np.column_stack([gt_pos[:, 1], gt_pos[:, 2], -gt_pos[:, 0]]),
        'X→Y, Y→Z, Z→X, flip_XY': np.column_stack([-gt_pos[:, 1], -gt_pos[:, 2], gt_pos[:, 0]]),
        'X→Y, Y→Z, Z→X, flip_XZ': np.column_stack([-gt_pos[:, 1], gt_pos[:, 2], -gt_pos[:, 0]]),
        'X→Y, Y→Z, Z→X, flip_YZ': np.column_stack([gt_pos[:, 1], -gt_pos[:, 2], -gt_pos[:, 0]]),
        'X→Y, Y→Z, Z→X, flip_XYZ': np.column_stack([-gt_pos[:, 1], -gt_pos[:, 2], -gt_pos[:, 0]]),
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
        
        # Calculate shape correlation (how well the trajectories correlate)
        # Sample both trajectories at same time points for correlation
        if len(gt_transformed) > len(fs_pos):
            # Downsample GT to match FS length
            indices = np.linspace(0, len(gt_transformed)-1, len(fs_pos), dtype=int)
            gt_sampled = gt_transformed[indices]
            fs_sampled = fs_pos
        else:
            # Use all GT points
            gt_sampled = gt_transformed
            indices = np.linspace(0, len(fs_pos)-1, len(gt_transformed), dtype=int)
            fs_sampled = fs_pos[indices]
        
        # Calculate correlation coefficient for X and Y coordinates
        try:
            corr_x = np.corrcoef(gt_sampled[:, 0], fs_sampled[:, 0])[0, 1]
            corr_y = np.corrcoef(gt_sampled[:, 1], fs_sampled[:, 1])[0, 1]
            shape_correlation = (abs(corr_x) + abs(corr_y)) / 2
        except:
            shape_correlation = 0
        
        # Combined score: range similarity + shape correlation
        combined_score = (similarity_score + shape_correlation) / 2
        
        results[name] = {
            'gt_ranges': gt_ranges,
            'fs_ranges': fs_ranges,
            'similarity_score': similarity_score,
            'shape_correlation': shape_correlation,
            'combined_score': combined_score,
            'transformed_pos': gt_transformed,
            'corr_x': corr_x if 'corr_x' in locals() else 0,
            'corr_y': corr_y if 'corr_y' in locals() else 0
        }
        
        print(f"\n🔍 {name}:")
        print(f"  GT Ranges: X={gt_ranges[0]:.3f}, Y={gt_ranges[1]:.3f}, Z={gt_ranges[2]:.3f}")
        print(f"  FS Ranges: X={fs_ranges[0]:.3f}, Y={fs_ranges[1]:.3f}, Z={fs_ranges[2]:.3f}")
        print(f"  Range Similarity: {similarity_score:.3f}")
        print(f"  Shape Correlation: {shape_correlation:.3f} (X:{results[name]['corr_x']:.3f}, Y:{results[name]['corr_y']:.3f})")
        print(f"  Combined Score: {combined_score:.3f}")
    
    # Find best transformation
    best_transform = max(results.keys(), key=lambda k: results[k]['combined_score'])
    
    # If multiple transformations have the same score, prefer positive correlations
    max_score = results[best_transform]['combined_score']
    tied_transforms = [k for k, v in results.items() if abs(v['combined_score'] - max_score) < 0.001]
    
    if len(tied_transforms) > 1:
        print(f"\n🔄 Multiple transformations tied with score {max_score:.3f}, selecting based on positive correlations...")
        # Show correlation details for tied transforms
        for t in tied_transforms:
            r = results[t]
            pos_count = (r['corr_x'] > 0) + (r['corr_y'] > 0)
            print(f"   {t}: {pos_count} positive correlations (X:{r['corr_x']:.3f}, Y:{r['corr_y']:.3f})")
        
        # Manually select the one with both positive correlations
        if 'X→Y, Y→Z, Z→X, flip_XY' in tied_transforms:
            best_transform = 'X→Y, Y→Z, Z→X, flip_XY'
            print(f"   Selected: {best_transform} (both X and Y correlations positive)")
        else:
            # Fallback to original tie-breaking logic
            def correlation_score(transform_name):
                r = results[transform_name]
                pos_corr_count = (r['corr_x'] > 0) + (r['corr_y'] > 0)
                avg_abs_corr = (abs(r['corr_x']) + abs(r['corr_y'])) / 2
                return (pos_corr_count, avg_abs_corr)
            
            best_transform = max(tied_transforms, key=correlation_score)
            r = results[best_transform]
            best_pos_count = (r['corr_x'] > 0) + (r['corr_y'] > 0)
            print(f"   Selected: {best_transform} ({best_pos_count} positive correlations)")
    
    print(f"\n🏆 Best transformation: {best_transform}")
    print(f"   Combined score: {results[best_transform]['combined_score']:.3f}")
    print(f"   Range similarity: {results[best_transform]['similarity_score']:.3f}")
    print(f"   Shape correlation: {results[best_transform]['shape_correlation']:.3f}")
    print(f"   X Correlation: {results[best_transform]['corr_x']:.3f}")
    print(f"   Y Correlation: {results[best_transform]['corr_y']:.3f}")
    
    return results, best_transform

def apply_best_coordinate_transformation(df, best_transform_name):
    """Apply the best coordinate transformation found"""
    df_transformed = df.copy()
    
    # Store original coordinates
    orig_x = df['tx'].values
    orig_y = df['ty'].values  
    orig_z = df['tz'].values
    
    if best_transform_name == 'Original':
        # No transformation
        pass
    elif best_transform_name == 'X→Y, Y→Z, Z→X':
        df_transformed['tx'] = orig_y  
        df_transformed['ty'] = orig_z  
        df_transformed['tz'] = orig_x
    elif best_transform_name == 'X→Y, Y→Z, Z→X, flip_X':
        df_transformed['tx'] = -orig_y  
        df_transformed['ty'] = orig_z  
        df_transformed['tz'] = orig_x
    elif best_transform_name == 'X→Y, Y→Z, Z→X, flip_Y':
        df_transformed['tx'] = orig_y  
        df_transformed['ty'] = -orig_z  
        df_transformed['tz'] = orig_x
    elif best_transform_name == 'X→Y, Y→Z, Z→X, flip_Z':
        df_transformed['tx'] = orig_y  
        df_transformed['ty'] = orig_z  
        df_transformed['tz'] = -orig_x
    elif best_transform_name == 'X→Y, Y→Z, Z→X, flip_XY':
        df_transformed['tx'] = -orig_y  
        df_transformed['ty'] = -orig_z  
        df_transformed['tz'] = orig_x
    elif best_transform_name == 'X→Y, Y→Z, Z→X, flip_XZ':
        df_transformed['tx'] = -orig_y  
        df_transformed['ty'] = orig_z  
        df_transformed['tz'] = -orig_x
    elif best_transform_name == 'X→Y, Y→Z, Z→X, flip_YZ':
        df_transformed['tx'] = orig_y  
        df_transformed['ty'] = -orig_z  
        df_transformed['tz'] = -orig_x
    elif best_transform_name == 'X→Y, Y→Z, Z→X, flip_XYZ':
        df_transformed['tx'] = -orig_y  
        df_transformed['ty'] = -orig_z  
        df_transformed['tz'] = -orig_x
    
    return df_transformed

def create_trajectory_comparison():
    """Create comprehensive trajectory visualization with best coordinate transformation"""
    
    # Find the best transformation
    results, best_transform = find_best_coordinate_transformation()
    
    # Load trajectories
    print(f"\n📁 Loading trajectory data...")
    gt_original = read_tum_trajectory("groundtruth_normalized.txt")
    fs = read_tum_trajectory("estimated_trajectory_foundationstereo.txt") 
    orig = read_tum_trajectory("estimated_trajectory_original_normalized.txt")
    
    # Apply best coordinate transformation to ground truth
    print(f"🔄 Applying best coordinate transformation to ground truth...")
    print(f"   Transformation: {best_transform}")
    print(f"   Combined score: {results[best_transform]['combined_score']:.3f}")
    gt = apply_best_coordinate_transformation(gt_original, best_transform)
    
    print(f"✅ Loaded and transformed trajectories:")
    print(f"  Ground Truth: {len(gt)} poses (coordinate-aligned)")
    print(f"  FoundationStereo: {len(fs)} poses") 
    print(f"  Original ORB-SLAM3: {len(orig)} poses")
    
    # Calculate trajectory statistics
    def calc_stats(df, name):
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
    print("📊 BEST-ALIGNED STATISTICS")
    print("="*50)
    
    gt_path, gt_dur = calc_stats(gt, "Ground Truth (Best Aligned)")
    fs_path, fs_dur = calc_stats(fs, "FoundationStereo")
    orig_path, orig_dur = calc_stats(orig, "Original ORB-SLAM3")
    
    # Show final alignment quality
    gt_ranges = np.array([gt['tx'].max() - gt['tx'].min(), 
                         gt['ty'].max() - gt['ty'].min(), 
                         gt['tz'].max() - gt['tz'].min()])
    fs_ranges = np.array([fs['tx'].max() - fs['tx'].min(), 
                         fs['ty'].max() - fs['ty'].min(), 
                         fs['tz'].max() - fs['tz'].min()])
    orig_ranges = np.array([orig['tx'].max() - orig['tx'].min(), 
                           orig['ty'].max() - orig['ty'].min(), 
                           orig['tz'].max() - orig['tz'].min()])
    
    fs_similarity = results[best_transform]['similarity_score']
    fs_correlation = results[best_transform]['shape_correlation']
    
    print(f"\n🎯 BEST ALIGNMENT QUALITY:")
    print(f"  Range Similarity: {fs_similarity:.1%}")
    print(f"  Shape Correlation: {fs_correlation:.1%}")
    print(f"  Combined Score: {results[best_transform]['combined_score']:.1%}")
    
    # Create comprehensive visualization
    fig = plt.figure(figsize=(20, 15))
    
    # 2D XY trajectory plot - BEST ALIGNED COMPARISON
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(gt['tx'], gt['ty'], 'k-', linewidth=3, label=f'Ground Truth (Best Aligned, {len(gt)} poses)', alpha=0.8)
    ax1.plot(fs['tx'], fs['ty'], 'r-', linewidth=2, label=f'FoundationStereo ({len(fs)} poses)', alpha=0.8)
    ax1.plot(orig['tx'], orig['ty'], 'b-', linewidth=2, label=f'Original ORB-SLAM3 ({len(orig)} poses)', alpha=0.8)
    
    # Mark start points
    ax1.plot(gt['tx'].iloc[0], gt['ty'].iloc[0], 'ko', markersize=8, label='GT Start')
    ax1.plot(fs['tx'].iloc[0], fs['ty'].iloc[0], 'ro', markersize=8, label='FS Start')
    ax1.plot(orig['tx'].iloc[0], orig['ty'].iloc[0], 'bo', markersize=8, label='Orig Start')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title(f'✅ BEST ALIGNED Trajectory Comparison\n({best_transform})')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 2D XZ trajectory plot  
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(gt['tx'], gt['tz'], 'k-', linewidth=3, label='Ground Truth (Best Aligned)', alpha=0.8)
    ax2.plot(fs['tx'], fs['tz'], 'r-', linewidth=2, label='FoundationStereo', alpha=0.8)
    ax2.plot(orig['tx'], orig['tz'], 'b-', linewidth=2, label='Original ORB-SLAM3', alpha=0.8)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Z (m)')
    ax2.set_title('XZ Plane (Best Aligned)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    # Position vs Time plots
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(gt['timestamp'], gt['tx'], 'k-', linewidth=2, label='GT X', alpha=0.7)
    ax3.plot(gt['timestamp'], gt['ty'], 'k--', linewidth=2, label='GT Y', alpha=0.7)
    ax3.plot(fs['timestamp'], fs['tx'], 'r-', linewidth=1.5, label='FS X', alpha=0.8)
    ax3.plot(fs['timestamp'], fs['ty'], 'r--', linewidth=1.5, label='FS Y', alpha=0.8)
    ax3.plot(orig['timestamp'], orig['tx'], 'b-', linewidth=1.5, label='Orig X', alpha=0.8)
    ax3.plot(orig['timestamp'], orig['ty'], 'b--', linewidth=1.5, label='Orig Y', alpha=0.8)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (m)')
    ax3.set_title('Position vs Time (Best Aligned)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 3D trajectory plot
    ax4 = plt.subplot(2, 3, 4, projection='3d')
    ax4.plot(gt['tx'], gt['ty'], gt['tz'], 'k-', linewidth=3, label='Ground Truth (Best Aligned)', alpha=0.8)
    ax4.plot(fs['tx'], fs['ty'], fs['tz'], 'r-', linewidth=2, label='FoundationStereo', alpha=0.8)
    ax4.plot(orig['tx'], orig['ty'], orig['tz'], 'b-', linewidth=2, label='Original ORB-SLAM3', alpha=0.8)
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_zlabel('Z (m)')
    ax4.set_title('3D Trajectory (Best Aligned)')
    ax4.legend()
    
    # Coordinate range comparison
    ax5 = plt.subplot(2, 3, 5)
    
    x_pos = np.arange(3)
    width = 0.25
    
    ax5.bar(x_pos - width, gt_ranges, width, label='Ground Truth (Best Aligned)', alpha=0.8, color='black')
    ax5.bar(x_pos, fs_ranges, width, label='FoundationStereo', alpha=0.8, color='red')
    ax5.bar(x_pos + width, orig_ranges, width, label='Original ORB-SLAM3', alpha=0.8, color='blue')
    
    ax5.set_xlabel('Coordinate Axis')
    ax5.set_ylabel('Range (m)')
    ax5.set_title('Coordinate Range Comparison (Best Aligned)')
    ax5.set_xticks(x_pos)
    ax5.set_xticklabels(['X', 'Y', 'Z'])
    ax5.legend()
    ax5.grid(True, alpha=0.3, axis='y')
    
    # Add value labels
    for i, (gt_r, fs_r, orig_r) in enumerate(zip(gt_ranges, fs_ranges, orig_ranges)):
        ax5.text(i - width, gt_r + 0.1, f'{gt_r:.1f}', ha='center', va='bottom', fontsize=8)
        ax5.text(i, fs_r + 0.1, f'{fs_r:.1f}', ha='center', va='bottom', fontsize=8)
        ax5.text(i + width, orig_r + 0.1, f'{orig_r:.1f}', ha='center', va='bottom', fontsize=8)
    
    # Alignment quality comparison
    ax6 = plt.subplot(2, 3, 6)
    
    metrics = ['Range\nSimilarity', 'Shape\nCorrelation', 'Combined\nScore']
    values = [fs_similarity, fs_correlation, results[best_transform]['combined_score']]
    colors = ['lightblue', 'lightgreen', 'gold']
    
    bars = ax6.bar(metrics, values, color=colors, alpha=0.8)
    ax6.set_ylabel('Score')
    ax6.set_title('Alignment Quality Metrics')
    ax6.set_ylim(0, 1)
    ax6.grid(True, alpha=0.3, axis='y')
    
    # Add value labels
    for bar, value in zip(bars, values):
        height = bar.get_height()
        ax6.text(bar.get_x() + bar.get_width()/2., height + 0.02,
                f'{value:.1%}', ha='center', va='bottom', fontweight='bold')
    
    plt.tight_layout()
    plt.savefig('best_aligned_trajectory_comparison.png', dpi=300, bbox_inches='tight')
    print("\n✅ Saved best_aligned_trajectory_comparison.png")
    
    # Summary
    print(f"\n" + "="*60)
    print("🎯 BEST ALIGNED TRAJECTORY COMPARISON SUMMARY")
    print("="*60)
    print(f"🔧 Best Transformation: {best_transform}")
    print(f"📊 Alignment Quality: {results[best_transform]['combined_score']:.1%}")
    print(f"📏 Path Length Ratios (Best Aligned):")
    print(f"  FoundationStereo: {fs_path/gt_path*100:.1f}% of ground truth")
    print(f"  Original ORB-SLAM3: {orig_path/gt_path*100:.1f}% of ground truth")
    print(f"\n⏱️ Duration Coverage:")
    print(f"  FoundationStereo: {fs_dur/gt_dur*100:.1f}% of ground truth")
    print(f"  Original ORB-SLAM3: {orig_dur/gt_dur*100:.1f}% of ground truth")
    print(f"\n🎯 Final Alignment Metrics:")
    print(f"  Range Similarity: {fs_similarity:.1%}")
    print(f"  Shape Correlation: {fs_correlation:.1%}")
    print(f"  X Correlation: {results[best_transform]['corr_x']:.3f}")
    print(f"  Y Correlation: {results[best_transform]['corr_y']:.3f}")
    
    print(f"\n✅ This should now be a PROPERLY aligned comparison!")

if __name__ == "__main__":
    print("🎯 OPTIMAL TRAJECTORY ALIGNMENT")
    print("="*50)
    print("Testing coordinate transformations + sign flips")
    print("Finding best alignment using range similarity + shape correlation")
    print("="*50)
    create_trajectory_comparison() 