#!/bin/bash

# Script to extract ROS bag data to mav0 format
# Usage: ./run_extraction.sh [output_directory]

# Activate the conda environment
source ~/miniconda3/etc/profile.d/conda.sh
conda activate foundation_stereo

# Default bag file path
BAG_FILE="/home/lunar/Downloads/lunar_stereo_data_2025-06-17-17-13-18.bag"

# Output directory (default if not provided)
OUTPUT_DIR=${1:-"./lunar_mav0_data"}

echo "Extracting bag: $BAG_FILE"
echo "Output directory: $OUTPUT_DIR"

# Run the extraction script
python extract_bag_to_mav0.py "$BAG_FILE" "$OUTPUT_DIR"

echo "Extraction complete!"
echo "Dataset structure:"
tree "$OUTPUT_DIR/mav0" -L 2 