import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import re

# Function to load and normalize data for each robot from a given CSV file
def load_and_normalize_data(csv_path):
    df = pd.read_csv(csv_path)
    df['timestamp'] -= df['timestamp'].min()
    return df

# Function to plot the trajectories with the true grayscale background
def plot_robot_trajectories_true_grayscale(df_list, background_image, extent, sheet_name):
    # Create a figure and a subplot
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title(f"Trajectories for {sheet_name}")

    # Display the background image in true grayscale
    ax.imshow(background_image.convert('L'), extent=extent, aspect='auto', cmap='gray')

    # For each dataframe (robot data) in the list
    for df in df_list:
        # Normalize the timestamp to start at 0
        df['timestamp'] -= df['timestamp'].iloc[0]

        # Create a scatter plot for the robot positions with a colormap based on the timestamp
        points = ax.scatter(df['position_x'], df['position_y'], c=df['timestamp'], cmap='viridis', s=20)

    # Add colorbar to the plot to represent the timestamp
    cbar = fig.colorbar(points, ax=ax)
    cbar.set_label('Time')

    # Set labels and axis limits
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    ax.set_xlim(extent[:2])
    ax.set_ylim(extent[2:])
    ax.grid(True)

    # Show the plot
    plt.show()

# Function to determine which map to use based on the experiment number in the file name
def select_map_image(file_name, map1_path, map2_path):
    experiment_number = int(re.search(r'experiment(\d+)', file_name).group(1))
    if experiment_number in [1, 3, 5, 6]:
        return Image.open(map1_path), 35.5  # Scaling factor for map1
    elif experiment_number in [2, 4]:
        return Image.open(map2_path), 42.0  # Scaling factor for map2
    else:
        raise ValueError(f"Unexpected experiment number: {experiment_number}")

# Paths to the CSV files
robot0_data_path = 'path_to_your_robot0_data.csv'
robot1_data_path = 'path_to_your_robot1_data.csv'
robot2_data_path = 'path_to_your_robot2_data.csv'

# Load and normalize the data
robot0_data = load_and_normalize_data(robot0_data_path)
robot1_data = load_and_normalize_data(robot1_data_path)
robot2_data = load_and_normalize_data(robot2_data_path)

# Combine the data into a list
robot_data_list = [robot0_data, robot1_data, robot2_data]

# Paths to the map images
map1_path = 'path_to_your_my_map.pgm'
map2_path = 'path_to_your_map_obstacles.pgm'

# Determine which map to use based on the experiment number in the file name
background_image, scale_height = select_map_image(robot0_data_path, map1_path, map2_path)

# Rescale the image
image_width, image_height = background_image.size
scaling_factor = scale_height / image_height
rescaled_background_image = background_image.resize(
    (int(image_width * scaling_factor), int(image_height * scaling_factor)),
    Image.ANTIALIAS
)
half_width = rescaled_background_image.size[0] / 2
half_height = rescaled_background_image.size[1] / 2
extent = [-half_width, half_width, -half_height, half_height]

# Plot the trajectories with the true grayscale background
plot_robot_trajectories_true_grayscale(robot_data_list, rescaled_background_image, extent, "Experiment 1 Repetition 2")
