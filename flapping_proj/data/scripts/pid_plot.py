import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
import os

data = pd.read_csv(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'pid_data.csv'))
# Trim away duplicate rows with time = 0

time_data = data['time']
altitude_target_data = data['altitude_target']
altitude_error_data = data['altitude_error']
altitude_output_data = data['altitude_output']

max_time = 7.194
# Trim away duplicate rows with time = 0
time_data_trimmed = time_data[(max_time >= time_data) & (time_data > 0.0001)].to_numpy()
altitude_target_data_trimmed = altitude_target_data[(max_time >= time_data) & (time_data > 0.0001)].to_numpy()
altitude_error_data_trimmed = altitude_error_data[(max_time >= time_data) & (time_data > 0.0001)].to_numpy()
altitude_output_data_trimmed = altitude_output_data[(max_time >= time_data) & (time_data > 0.0001)].to_numpy()
altitude_current = (altitude_target_data_trimmed - altitude_error_data_trimmed)


#flying input sin signal
amplitude = 2.758
input_signal = np.sin(time_data_trimmed * 2 * np.pi * 0.5)
#add pid output to input signal
for i in range(len(altitude_output_data_trimmed)):
    if altitude_output_data_trimmed[i] < 0:
        input_signal[i] *= amplitude * .3
    else:
        input_signal[i] *= (amplitude + altitude_output_data_trimmed[i]) 


# Static subplot figure
fig_static, axes_static = plt.subplots(2, 1)

# Subplot 1: Altitude target and current altitude
axes_static[0].plot(time_data_trimmed, altitude_target_data_trimmed, label='Altitude Target')
axes_static[0].plot(time_data_trimmed, altitude_current, label='Current Altitude')
axes_static[0].set_ylabel('Altitude')
axes_static[0].set_title('Altitude Target vs Current Altitude')
axes_static[0].legend()

# Subplot 2: Altitude error and altitude output
# axes_static[1].plot(time_data_trimmed, altitude_error_data_trimmed, label='Altitude Error')
# axes_static[1].plot(time_data_trimmed, altitude_output_data_trimmed, label='Altitude Output')
axes_static[1].plot(time_data_trimmed, input_signal, label='Input Torque Signal')
axes_static[1].set_xlabel('Time')
axes_static[1].set_ylabel('Input Torque (Nm)')
axes_static[1].set_title('Input Signal')
#set green color
axes_static[1].lines[0].set_color('green')
# axes_static[1].legend()

# Animated subplot figure
fig_animated, axes_animated = plt.subplots(2, 1)

# Set the initial y-axis limits for animation
axes_animated[0].set_ylim(-1.2, 1.2)
axes_animated[1].set_ylim(-0.5, 0.5)

# Initialize empty lines to update in the animation
line_target, = axes_animated[0].plot([], [], label='Altitude Target')
line_current, = axes_animated[0].plot([], [], label='Current Altitude')
line_input, = axes_animated[1].plot([], [], label='Input Torque Signal')

# Define the initialization function for the animation
def init():
    line_target.set_data([], [])
    line_current.set_data([], [])
    line_input.set_data([], [])
    return line_target, line_current, line_input

# Define the update function for the animation
skip = 300
frames = len(time_data_trimmed) // skip #skip since too much data
def update(frame):
    frame *= skip
    x = time_data_trimmed[:frame:skip]
    y_target = altitude_target_data_trimmed[:frame:skip]
    y_current = altitude_current[:frame:skip]
    y_input = input_signal[:frame:skip]
    # y_output = altitude_output_data_trimmed[:frame:skip]
    
    line_target.set_data(x, y_target)
    line_current.set_data(x, y_current)
    line_input.set_data(x, y_input)

    # axes_animated[0].relim()
    # axes_animated[0].autoscale_view()
    # axes_animated[1].relim()
    # axes_animated[1].autoscale_view()

    return line_target, line_current, line_input

x_min_static, x_max_static = min(time_data_trimmed), max(time_data_trimmed)
y_min_static1, y_max_static1 = min(altitude_current), max(altitude_current)
y_min_static2, y_max_static2 = min(min(altitude_output_data_trimmed), min(altitude_error_data_trimmed)), max(max(altitude_output_data_trimmed), max(altitude_error_data_trimmed))
axes_animated[0].set_xlim(x_min_static, x_max_static)
axes_animated[1].set_xlim(x_min_static, x_max_static)
axes_animated[0].set_ylim(y_min_static1, y_max_static1 * 1.05)
axes_animated[1].set_ylim(y_min_static2, y_max_static2 * 1.05)
#axis labels
axes_animated[0].set_xlabel('Time (s)')
axes_animated[0].set_ylabel('Altitude (m)')
axes_animated[1].set_xlabel('Time (s)')
axes_animated[1].set_ylabel('Altitude (s)')
# Create the animation
duration_s = max_time
interval = duration_s * 1000 / frames # 1 min animation into ms / number of data points
print(frames)
ani = FuncAnimation(fig_animated, update, frames=frames, init_func=init, blit=True, interval=interval)

# Add legends to the subplots of the animated figure
axes_animated[0].legend()
axes_animated[1].legend()

# Show both figures
plt.show()
ani.save('pid_animation.gif', writer='pillow')