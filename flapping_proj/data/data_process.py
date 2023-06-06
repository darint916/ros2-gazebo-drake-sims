import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

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
def quaternion_to_euler(x, y, z, w):
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

    return roll_x, pitch_y, yaw_z

# Create the 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initialize the quiver objects
arrow = ax.quiver(0, 0, 0, 0, 0, 0, color='r', label='Model', alpha=0.5)

# Update function for animation
def update(frame):
    ax.cla()  # Clear the current axes

    # Plot the trajectory
    ax.plot(position_x[:frame+1], position_y[:frame+1], position_z[:frame+1], marker='o')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Object Trajectory')

    # Calculate Euler angles
    roll, pitch, yaw = quaternion_to_euler(quaternion_x[frame], quaternion_y[frame], quaternion_z[frame], quaternion_w[frame])

    # Update quiver objects
    arrow.set_UVC(roll, pitch, yaw)
    arrow.set_offsets([position_x[frame], position_y[frame], position_z[frame]])
# Create animation
animation = FuncAnimation(fig, update, frames=len(time), interval=100, blit=False)

animation.save('animation.mp4', writer='pillow')

# Create synchronized plots for joint angles
fig2, ax2 = plt.subplots(2, 2, sharex='col')

ax2[0, 0].plot(time, joint_LW_J_Pitch, label='LW_J_Pitch')
ax2[0, 0].set_ylabel('Angle (radians)')
ax2[0, 0].legend()

ax2[0, 1].plot(time, joint_RW_J_Pitch, label='RW_J_Pitch')
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
