import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

# Read the CSV file
data = []
with open('data.csv', 'r') as file:
    csv_reader = csv.reader(file)
    next(csv_reader)  # Skip the header row
    for row in csv_reader:
        data.append([float(val) for val in row])

# Extract the data columns
time = np.array([row[0] for row in data])
current_x = np.array([row[1] for row in data])
current_y = np.array([row[2] for row in data])
angle_orientation = np.array([row[3] for row in data])
target_x = np.array([row[4] for row in data])
target_y = np.array([row[5] for row in data])

# Create the figure and axes
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(np.min(current_x) - 1, np.max(current_x) + 1)
ax.set_ylim(np.min(current_y) - 1, np.max(current_y) + 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Robot Trajectory')

# Initialize the line objects
target_line, = ax.plot([], [], 'ro', label='Target Points')
current_line, = ax.plot([], [], 'b.', label='Current Points')
trajectory_line, = ax.plot([], [], 'k--', label='Robot Trajectory')

# Initialize the arrow object for angle orientation
arrow = mpatches.FancyArrowPatch((0, 0), (0, 0), color='C2')
ax.add_patch(arrow)

# Update function for animation
def update(frame):
    target_line.set_data(target_x[:frame+1], target_y[:frame+1])
    current_line.set_data(current_x[:frame+1], current_y[:frame+1])
    trajectory_line.set_data(current_x[:frame+1], current_y[:frame+1])
    arrow.set_positions((current_x[frame], current_y[frame]),
                        (current_x[frame] + 0.5 * np.cos(angle_orientation[frame]),
                         current_y[frame] + 0.5 * np.sin(angle_orientation[frame])))
    return target_line, current_line, trajectory_line, arrow

# Create the animation
legend_handles = [target_line, current_line, trajectory_line, arrow]
legend_labels = ['Target Points', 'Current Points', 'Robot Trajectory', 'Angle Orientation']
plt.legend(loc='lower left')
plt.grid(True)

animation = FuncAnimation(fig, update, frames=len(data), interval=100, blit=True)
# Save the animation as a GIF
animation.save('plots/robot_trajectory3.gif', writer='pillow')

# Display the plot
plt.axis('equal')
plt.tight_layout()
plt.show()
