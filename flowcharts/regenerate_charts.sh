#!/bin/bash

# Regenerate all flowchart PNG files from their DOT file sources
# Place resulting PNG files in the png_files directory

# Navigate to the flowcharts directory (assuming script is run from there)
cd "$(dirname "$0")"

# Create output directory if it doesn't exist
mkdir -p png_files

# Process each DOT file and generate corresponding PNG
for dot_file in dot_files/*.dot; do
  # Extract just the filename without path or extension
  base_name=$(basename "$dot_file" .dot)
  echo "Processing $base_name..."
  
  # Generate PNG file in png_files directory
  dot -Tpng "$dot_file" -o "png_files/${base_name}.png"
  
  if [ $? -eq 0 ]; then
    echo "✅ Successfully generated png_files/${base_name}.png"
  else
    echo "❌ Error generating PNG for $dot_file"
  fi
done

echo "All flowcharts regenerated!" 