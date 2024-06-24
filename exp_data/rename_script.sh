#!/bin/bash

# Directory where the files are located
DIRECTORY="/home/alvaro/ros2_humble_wss/thesis_wsp/exp_data"

# Iterate over all files starting with "exp_data_fp.launch_experiment"
for file in "$DIRECTORY"/exp_data_nofp.launch_experiment*; do
  # Get the base name of the file (without the path)
  base_name=$(basename "$file")
  # Replace "exp_data_fp.launch_experiment" with "exp_data_fp.launch.py_experiment" in the file name
  new_name=${base_name/exp_data_nofp.launch_experiment/exp_data_nofp.launch.py_experiment}
  # Rename the file
  mv "$file" "$DIRECTORY/$new_name"
done
