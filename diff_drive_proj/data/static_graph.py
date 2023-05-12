import csv
import numpy as np
import matplotlib.pyplot as plt

# Read the CSV file
data = []
with open('data.csv', 'r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        data.append([float(val) for val in row])

# Extract the data columns
time = np.array([row[0] for row in data])
current_x = np.array([row[1] for row in data])
current_y = np.array([row[2] for row in data])
target_x = np.array([row[4] for row in data])
target_y = np.array([row[5] for row in data])
angular_velocity_left = np.array([row[6] for row in data])
angular_velocity_right = np.array([row[7] for row in data])

# Plot the Lissajous curve
A = 5.0
B = 5.0
a = 3.0
b = 2.0
delta = np.pi / 2.0
t = np.linspace(0, 2*np.pi, 1000)
lissajous_x = A * np.sin(a * t + delta)
lissajous_y = B * np.sin(b * t)

plt.figure(figsize=(8, 6))
plt.plot(lissajous_x, lissajous_y, 'b-', label='Lissajous Curve')
plt.plot(target_x, target_y, 'ro', label='Target Points')
plt.plot(current_x, current_y, 'g.', label='Current Points')
plt.plot(current_x, current_y, 'k--', label='Robot Trajectory')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Lissajous Curve and Robot Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('plots/robot_traj3.png')

# Plot velocities over time
plt.figure(figsize=(8, 4))
plt.plot(time, angular_velocity_left, 'r-', label='Angular Velocity Left')
plt.plot(time, angular_velocity_right, 'b-', label='Angular Velocity Right')
plt.xlabel('Time')
plt.ylabel('Angular Velocity')
plt.title('Angular Velocities over Time')
plt.legend()
plt.grid(True)
plt.savefig('plots/angular_velocities3.png')

# Display the plots
plt.show()