import os
import json
import numpy as np
import matplotlib.pyplot as plt

# Set the base directory where the folders are located
base_dir = os.path.join(os.path.dirname(__file__), 'data')

# Prepare lists to store data for plotting
frequencies = []
param1 = []  # Stroke Joint Stiffness
param2 = []  # Pitch Joint Stiffness
param3 = []  # Stroke Joint Damping
costs = []

for i in range(11, 34):  # 10 Hz to 20 Hz
    if i == 15 or i == 16: continue
    folder_name = f"gamma_sweep_{i}_hz"
    file_path = os.path.join(base_dir, folder_name, 'data.json')
    
    # Read JSON data from the file
    with open(file_path, 'r') as file:
        data = json.load(file)
        top_result = sorted(data, key=lambda x: x['cost'], reverse=False)[:4] #lowest cost
        frequencies.append(i)
        frequencies.append(i)
        frequencies.append(i)
        frequencies.append(i)
        for i, result in enumerate(top_result):
            best_cost = result['cost']
            opt_params = result['opt_params']
            print(f"Cost: {round(best_cost / 2, 3)}")
            print(f"Optimized Parameters: {opt_params}")
            param1.append(opt_params[0])
            param2.append(opt_params[1])
            param3.append(opt_params[2])
            costs.append(best_cost)

# Plotting the parameters
fig, axs = plt.subplots(2, 2, figsize=(10, 15))

# Subplots for each parameter
axs[0, 0].scatter(frequencies, param1, marker='o', label='Stroke Joint Stiffness (N/m)')
axs[0, 0].set_title('Stroke Joint Stiffness vs Frequency')
axs[0, 0].set_xlabel('Frequency (Hz)')
axs[0, 0].set_ylabel('Stiffness (N/m)')
axs[0, 0].grid(True)

axs[0, 1].scatter(frequencies, param2, marker='o', color='red', label='Pitch Joint Stiffness (N/m)')
axs[0, 1].set_title('Pitch Joint Stiffness vs Frequency')
axs[0, 1].set_xlabel('Frequency (Hz)')
axs[0, 1].set_ylabel('Stiffness (N/m)')
axs[0, 1].grid(True)

axs[1, 0].scatter(frequencies, param3, marker='o', color='green', label='Stroke Joint Damping (Ns/m)')
axs[1, 0].set_title('Stroke Joint Damping vs Frequency')
axs[1, 0].set_xlabel('Frequency (Hz)')
axs[1, 0].set_ylabel('Damping (Ns/m)')
axs[1, 0].grid(True)

lifts = [cost * -100 for cost in costs]
axs[1, 1].scatter(frequencies, lifts, marker='o', color='purple', label='Lift')
axs[1, 1].set_title('Cost vs Frequency')
axs[1, 1].set_xlabel('Frequency (Hz)')
axs[1, 1].set_ylabel('Lift (g)')
axs[1, 1].grid(True)

# Heatmap or another creative plot (optional)
# axs[3].imshow(...)  # You can fill this in with a heatmap or other plot based on more analysis

# Display the plots
# plt.tight_layout()
plt.show()
plt.grid(True)