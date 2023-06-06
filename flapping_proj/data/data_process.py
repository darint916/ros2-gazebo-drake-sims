import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np
# Read the data from the CSV file
data = pd.read_csv('data.csv')

# Extract the required columns
time = data['time']
position_x = data['position_x']
position_y = data['position_y']
position_z = data['position_z']
quaternion_x = data['quaternion_x']
quaternion_y = data['quaternion_y']
quaternion_z = data['quaternion_z']
quaternion_w = data['quaternion_w']
joint_LW_J_Pitch = data['joint_LW_J_Pitch']
joint_RW_J_Pitch = data['joint_RW_J_Pitch']
joint_LW_J_Flap = data['joint_LW_J_Flap']
joint_RW_J_Flap = data['joint_RW_J_Flap']

# Convert quaternion to Euler angles
def quaternion_to_euler(x, y, z, w,length=.2):
    import math
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x * length, pitch_y * length, yaw_z * length

def quiver_data_to_segments(X, Y, Z, u, v, w, length=1):
    segments = (X, Y, Z, X+v*length, Y+u*length, Z+w*length)
    segments = np.array(segments).reshape(6,-1)
    return [[[x, y, z], [u, v, w]] for x, y, z, u, v, w in zip(*list(segments))]

# Create the 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax = fig.add_subplot(122, projection='3d', sharex=ax, sharey=ax, sharez=ax)
# Initialize the quiver objects
roll, pitch, yaw = quaternion_to_euler(quaternion_x[0], quaternion_y[0], quaternion_z[0], quaternion_w[0])
quivers = ax.quiver(position_x[0], position_y[0], position_z[0], roll, pitch, yaw, color='r', label='Model', alpha=0.5)
ax.plot(position_x[0], position_y[0], position_z[0], marker='o')

min_x = np.min(position_x)
max_x = np.max(position_x)
x_offset = (max_x - min_x) / 15
min_y = np.min(position_y)
max_y = np.max(position_y)
y_offset = (max_y - min_y) / 15
min_z = np.min(position_z)
max_z = np.max(position_z)
z_offset = (max_z - min_z) / 15
# Update function for animation
def update(frame):
    ax.cla()  # Clear the current axes

    # Plot the trajectory
    ax.plot(position_x[:frame+1], position_y[:frame+1], position_z[:frame+1], marker='o', markersize=4, linestyle='dashed')


    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Object Trajectory')
    ax.set_xlim3d(min_x - x_offset, max_x + x_offset)
    ax.set_ylim3d(min_y - y_offset, max_y + y_offset)
    ax.set_zlim3d(min_z - z_offset, max_z + z_offset)
    

    # Calculate Euler angles
    roll, pitch, yaw = quaternion_to_euler(quaternion_x[frame], quaternion_y[frame], quaternion_z[frame], quaternion_w[frame])
    # quivers = ax.quiver(position_x[0], position_y[0], position_z[0], roll, pitch, yaw, color='r', label='Model', alpha=0.5)
    
    quivers = ax.quiver(position_x[frame], position_y[frame], position_z[frame], roll, pitch, yaw, color='r', label='Model',linestyle='--', alpha=0.5, length=.2, normalize=True,)

    # segments = quiver_data_to_segments(position_x[frame], position_y[frame], position_z[frame], roll, pitch, yaw, length=.3)
    # quivers.set_segments(segments)

    # Update quiver objects
    # arrow.set_UVC(arrow_direction[0], arrow_direction[1], arrow_direction[2])
    # arrow.set_offsets([position_x[frame], position_y[frame], position_z[frame]])
    return quivers
# Create animation
animation = FuncAnimation(fig, update, frames=len(time), interval=100, blit=False)

animation.save('animation.gif', writer='pillow')

# Create synchronized plots for joint angles
fig2, ax2 = plt.subplots(2, 2, sharex='col')

ax2[0, 0].plot(time, joint_LW_J_Pitch, label='LW_J_Pitch')
ax2[0, 0].set_xlabel('Time (s)')
ax2[0, 0].set_ylabel('Angle (radians)')
ax2[0, 0].legend()

ax2[0, 1].plot(time, joint_RW_J_Pitch, label='RW_J_Pitch')
ax2[0, 1].set_xlabel('Time (s)')
ax2[0, 1].set_ylabel('Angle (radians)')
ax2[0, 1].legend()

ax2[1, 0].plot(time, joint_LW_J_Flap, label='LW_J_Flap')
ax2[1, 0].set_xlabel('Time (s)')
ax2[1, 0].set_ylabel('Angle (radians)')
ax2[1, 0].legend()

ax2[1, 1].plot(time, joint_RW_J_Flap, label='RW_J_Flap')
ax2[1, 1].set_xlabel('Time (s)')
ax2[1, 1].set_ylabel('Angle (radians)')
ax2[1, 1].legend()

# Save the synchronized plots as an image file
fig2.savefig('joint_angles.png')

# Show the plots
plt.show()
