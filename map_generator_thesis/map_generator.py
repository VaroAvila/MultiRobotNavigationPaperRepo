import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.image as mpimg
from scipy.ndimage import rotate

# Variables to control offsets and file paths
x_offset = -20 #-24.2
y_offset = -20
rotation_offset = -180 #-13

csv_files = [
    '/home/alvaro/projects/map_generator_thesis/exp_data_fp.launch.py_experiment3_repetition3_robot0.csv',
    '/home/alvaro/projects/map_generator_thesis/exp_data_fp.launch.py_experiment3_repetition3_robot1.csv',
    '/home/alvaro/projects/map_generator_thesis/exp_data_fp.launch.py_experiment3_repetition3_robot2.csv'
]

background_image_path = '/home/alvaro/projects/map_generator_thesis/my_map.pgm'

# Function to parse lines from the CSV
def parse_line(line):
    parts = line.split('=')
    return {
        'timestamp': float(parts[0].strip().split()[0].replace(',', '.')),
        'position_x': float(parts[1].split()[0].strip().replace(',', '.')),
        'position_y': float(parts[2].split()[0].strip().replace(',', '.')),
        'position_z': float(parts[3].split()[0].strip().replace(',', '.')),
        'orientation_x': parts[4].strip(),
        'orientation_y': parts[5].strip(),
        'orientation_z': parts[6].strip(),
        'orientation_w': parts[7].strip()
    }

# Load the trajectory data
data = []
for csv_file in csv_files:
    with open(csv_file, 'r') as file:
        lines = file.readlines()[1:]  # Skip the header line
        data.extend([parse_line(line) for line in lines])
df = pd.DataFrame(data)

# Load the map image
background_image = mpimg.imread(background_image_path)

# Rotate the map image by the rotation offset to fix rotational offset
rotated_image = rotate(background_image, rotation_offset, reshape=True)

# Flip the image to correct the mirroring effect
flipped_image = np.fliplr(rotated_image)

# Initialize the plot
fig, ax = plt.subplots()

# Resolution and origin
resolution = 0.03
origin = [x_offset, y_offset, 0]

# Correct the extent calculation considering the rotated image
image_extent = [origin[0], origin[0] + flipped_image.shape[1] * resolution,
                origin[1], origin[1] + flipped_image.shape[0] * resolution]

# Display the flipped background image
ax.imshow(flipped_image, cmap='gray', extent=image_extent, origin='lower')

# Process and plot each CSV file's trajectory
for i, csv_file in enumerate(csv_files, start=1):
    with open(csv_file, 'r') as file:
        lines = file.readlines()[1:]  # Skip the header line
        data = [parse_line(line) for line in lines]
    
    df = pd.DataFrame(data)
    
    # Normalize the timestamp
    df['timestamp'] = df['timestamp'] - df['timestamp'].min()
    
    # Create a color map based on time
    norm = plt.Normalize(df['timestamp'].min(), df['timestamp'].max())
    cmap = plt.get_cmap('viridis')
    
    # Plot the trajectory with a gradient
    points = np.array([df['position_x'], df['position_y']]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(df['timestamp'])
    lc.set_linewidth(4)
    line = ax.add_collection(lc)
    
    # Mark the starting position
    ax.scatter(df['position_x'][0], df['position_y'][0], marker='o', s=400, color='black')
    ax.text(df['position_x'][0], df['position_y'][0], f'S{i}', fontsize=12, ha='center', va='center', color='white', weight='bold')

# Set plot limits
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)

# Set plot labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Robot Trajectories')
ax.set_aspect('equal', adjustable='box')

# Add a color bar
cbar = plt.colorbar(line, ax=ax)
cbar.set_label('Time')

# Save plot to file instead of showing
plt.savefig('/home/alvaro/projects/map_generator_thesis/robot_trajectories.png')

# Optionally, close the plot to free memory
plt.close()
