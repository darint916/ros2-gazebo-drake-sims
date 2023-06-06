import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Generate some random vector field data
x, y, z = np.meshgrid(np.linspace(-5, 5, 10), np.linspace(-5, 5, 10), np.linspace(-5, 5, 10))
u = np.sin(x) * np.cos(y) * np.cos(z)
v = -np.cos(x) * np.sin(y) * np.cos(z)
w = np.sqrt(2.0 / 3.0) * np.cos(x) * np.cos(y) * np.sin(z)

# Plot the vector field
ax.quiver(x, y, z, u, v, w)

# Set plot limits and labels
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([-5, 5])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()