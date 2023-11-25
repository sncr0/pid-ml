#!/bin/bash

folder_path="/home/samuel/pid-ml/cpp/src/results/"

# Generate a temporary gnuplot script
temp_script=$(mktemp)
echo "set term png; set output 'combined_plot.png'; plot \\" > "$temp_script"

for file_path in "$folder_path"/*.csv; do
    if [ -f "$file_path" ]; then
        # Extract filename without extension
        filename=$(basename -- "$file_path")
        filename_no_ext="${filename%.*}"

        echo "Processing $filename..."

        # Append the current file to the gnuplot script
        echo "'$file_path' every ::3 using 1:2 with lines title '$filename_no_ext', \\" >> "$temp_script"
    else
        echo "File not found: $file_path"
    fi
done

# Remove the trailing comma and execute the gnuplot script
sed -i '$ s/\\//' "$temp_script"
gnuplot "$temp_script"

# Clean up the temporary script
rm "$temp_script"