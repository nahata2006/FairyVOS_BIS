#!/usr/bin/env python3

def normalize_ground_truth_timestamps(input_file, output_file):
    """
    Normalize ground truth timestamps to start from 0, matching ORB-SLAM3 format
    """
    print(f"🔧 Normalizing timestamps in {input_file}...")
    
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    # Find the first timestamp
    first_timestamp = None
    for line in lines:
        line = line.strip()
        if line and not line.startswith('#'):
            parts = line.split()
            if len(parts) >= 8:
                first_timestamp = float(parts[0])
                break
    
    if first_timestamp is None:
        print("❌ No valid timestamps found!")
        return False
    
    print(f"📅 First timestamp: {first_timestamp}")
    print(f"🔄 Normalizing to start from 0.000000000...")
    
    # Normalize timestamps
    normalized_lines = []
    for line in lines:
        line = line.strip()
        if line and not line.startswith('#'):
            parts = line.split()
            if len(parts) >= 8:
                # Normalize timestamp
                timestamp = float(parts[0])
                normalized_timestamp = timestamp - first_timestamp
                
                # Reconstruct line with normalized timestamp
                parts[0] = f"{normalized_timestamp:.9f}"
                normalized_lines.append(' '.join(parts))
        elif line.startswith('#'):
            normalized_lines.append(line)
    
    # Write normalized file
    with open(output_file, 'w') as f:
        for line in normalized_lines:
            f.write(line + '\n')
    
    print(f"✅ Normalized ground truth saved to: {output_file}")
    print(f"📊 Processed {len(normalized_lines)} entries")
    
    # Show first few normalized timestamps
    print("\n📍 First 5 normalized timestamps:")
    for i, line in enumerate(normalized_lines[:5]):
        if not line.startswith('#'):
            timestamp = line.split()[0]
            print(f"  {i+1}: {timestamp}")
    
    return True

def main():
    print("🎯 TIMESTAMP ALIGNMENT FIX")
    print("=" * 50)
    print("Problem: Ground truth uses Unix timestamps, ORB-SLAM3 uses normalized timestamps")
    print("Solution: Normalize ground truth to start from 0")
    print("=" * 50)
    
    # Normalize ground truth
    success = normalize_ground_truth_timestamps("groundtruth.txt", "groundtruth_normalized.txt")
    
    if success:
        print("\n✅ Timestamp normalization complete!")
        print("📁 Use 'groundtruth_normalized.txt' for evaluation")
    else:
        print("\n❌ Timestamp normalization failed!")

if __name__ == "__main__":
    main() 