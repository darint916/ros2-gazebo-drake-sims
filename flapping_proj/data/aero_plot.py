import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.ticker as ticker
import os
import sys
import shutil
from matplotlib.animation import FuncAnimation


csv_name = sys.argv[2] if len(sys.argv) == 3 else 'aero.csv'

if len(sys.argv) < 2:
    print("Usage: python3 data_process.py <folder_name> <optional:csv_name>")
    print("No folder path provided, data will be saved in current directory")
    folder_name = ''
else:
    folder_name = sys.argv[1]
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)


# Step 1: Read the CSV file skipping the first 4 rows
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_source = os.path.join(script_dir, csv_name)
csv_dest = os.path.join(script_dir, folder_name, 'data', csv_name)
print("csv source", csv_source)
print("csv dest", csv_dest)

dest_folder = os.path.dirname(csv_dest)
if not os.path.exists(dest_folder):
    os.makedirs(dest_folder)

shutil.copy2(csv_source, csv_dest)
# print(df.columns)

folder_name = folder_name + '/aero_plots'
if not os.path.exists(folder_name):
    os.makedirs(folder_name)
# Step 2: Create separate plots for each blade number
df = pd.read_csv(csv_name, skiprows=range(1,30)) #skip first wing*blade rows starting at 1
unique_wing_names = df['wing_name'].unique()
unique_blade_numbers = df['blade_number'].unique()
# print(df.head(3))
for wing_name in unique_wing_names:
    blade_data = df[(df['wing_name'] == wing_name)]
    blade_force_x = blade_data['blade_force_x'].to_numpy()
    blade_force_y = blade_data['blade_force_y'].to_numpy()
    blade_force_z = blade_data['blade_force_z'].to_numpy()
    blade_force_magnitude = np.sqrt(blade_force_x ** 2 + blade_force_y ** 2 + blade_force_z ** 2)
    time = blade_data['time'].to_numpy()
    combined_f = {}
    for t, force in zip(time, blade_force_magnitude):
        combined_f[t] = combined_f.get(t, 0) + force 
    time = np.array(list(combined_f.keys()))
    blade_force_magnitude = np.array(list(combined_f.values()))
    fig = plt.figure()
    print("force magnitudes", blade_force_magnitude[:20])
    fig = plt.figure()
    plt.plot(time, blade_force_magnitude)
    plt.xlabel('Time (s)')
    plt.ylabel('Blade Force Magnitude (Nm)')
    plt.title(f'{wing_name}')
    plt.grid(True)
    fig.savefig(folder_name + '/force_mag_' + str(wing_name) + '.png')
    plt.show(block=False)

    mean_val = np.mean(blade_force_magnitude)
    with open(folder_name + "aero_data.txt", 'w') as file:
        file.write("mean force magnitude: ")
        file.write(str(mean_val) + "\n")
    

    combined_f_z = {}
    for t, force in zip(time, blade_force_z):
        combined_f_z[t] = combined_f_z.get(t, 0) + force
    time = np.array(list(combined_f_z.keys()))
    blade_force_z = np.array(list(combined_f_z.values()))
    
    mean_val = np.mean(blade_force_z)
    abs_mean_val = np.mean(np.abs(blade_force_z))
    with open(folder_name + "aero_data.txt", 'a') as file:
        file.write("mean force z: ")
        file.write(str(mean_val))
        file.write("\n")
        file.write("mean abs force z: ")
        file.write(str(abs_mean_val))
        file.write("\n")
    
    fig = plt.figure()
    plt.plot(time, blade_force_z)
    plt.xlabel('Time (s)')
    plt.ylabel('Blade Force Z (Nm)')
    plt.title(f'{wing_name}')
    plt.grid(True)
    fig.savefig(folder_name + '/force_z_' + str(wing_name) + '.png')
    plt.show()

    #get y force 
    combined_f_y = {}
    for t, force in zip(time, blade_force_y):
        combined_f_y[t] = combined_f_y.get(t, 0) + force
    time = np.array(list(combined_f_y.keys()))
    blade_force_y = np.array(list(combined_f_y.values()))

    mean_val = np.mean(blade_force_y)
    abs_mean_val = np.mean(np.abs(blade_force_y))
    with open(folder_name + "aero_data.txt", 'a') as file:
        file.write("mean force y: ")
        file.write(str(mean_val))
        file.write("\n")
        file.write("mean abs force y: ")
        file.write(str(abs_mean_val))
        file.write("\n")
    
    #get x force
    combined_f_x = {}
    for t, force in zip(time, blade_force_x):
        combined_f_x[t] = combined_f_x.get(t, 0) + force
    time = np.array(list(combined_f_x.keys()))
    blade_force_x = np.array(list(combined_f_x.values())) 
    
    mean_val = np.mean(blade_force_x)
    abs_mean_val = np.mean(np.abs(blade_force_x))
    with open(folder_name + "aero_data.txt", 'a') as file:
        file.write("mean force x: ")
        file.write(str(mean_val))
        file.write("\n")
        file.write("mean abs force x: ")
        file.write(str(abs_mean_val))
        file.write("\n")

    
    fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))
    #create arrow for each time point
    vectors = np.array([blade_force_x, blade_force_y, blade_force_z]).T.reshape(-1, 3)
    
    print("first 10 vectors", vectors[100:110])
    def get_arrow(t):
        x = 0
        y = 0
        z = 0
        u = vectors[t, 0]
        v = vectors[t, 1]
        w = vectors[t, 2]
        return x, y, z, u, v, w
    quiver = ax.quiver(*get_arrow(0))
    ax.set_xlim(min(blade_force_x), max(blade_force_x))
    ax.set_ylim(min(blade_force_y), max(blade_force_y))
    ax.set_zlim(min(blade_force_z), max(blade_force_z))
    def update(t):
        global quiver
        quiver.remove()
        quiver = ax.quiver(*get_arrow(t))
    ani = FuncAnimation(fig, update, frames=100, interval=100)
    ani.save(folder_name + '/animation_' + str(wing_name) + '.gif', writer='pillow')
    plt.show()


# for wing_name in unique_wing_names:
#     print("wing name", wing_name)
#     for blade_number in unique_blade_numbers:
#         # blade_data = df[(df['wing_name'] == wing_name) & (df['blade_number'] == blade_number)]
#         blade_data = df[(df['wing_name'] == wing_name)]
#         blade_force_x = blade_data['blade_force_x']
#         blade_force_y = blade_data['blade_force_y']
#         blade_force_z = blade_data['blade_force_z'].to_numpy()
#         blade_velocity_x = blade_data['blade_velocity_x'].to_numpy()
#         blade_velocity_y = blade_data['blade_velocity_y'].to_numpy()
#         blade_velocity_z = blade_data['blade_velocity_z'].to_numpy()
#         blade_velocity_magnitude = np.sqrt(blade_velocity_x ** 2 + blade_velocity_y ** 2 + blade_velocity_z ** 2)
#         blade_force_magnitude = np.sqrt(blade_force_x ** 2 + blade_force_y ** 2 + blade_force_z ** 2)
#         time = blade_data['time'].to_numpy()
#         combined_f = {}
#         for t, force in zip(time, blade_force_magnitude):
#             combined_f[t] = combined_f.get(t, 0) + force 
#         time = np.array(list(combined_f.keys()))
#         blade_force_magnitude = np.array(list(combined_f.values()))
#         # print("time first 10 rows", time[:10])
#         # print("asdf", time[-1] )
#         # print("asdf", len(time))
#         fft_result = np.fft.fft(blade_force_z)
#         # fft_result = np.fft.fft(blade_force_magnitude)
#         print("fft res len", len(fft_result))
#         # print("z force len", len(blade_force_z))
#         print("time len", int(time[-1] / .0001) - 2)
#         # frequencies = np.fft.fftfreq(int(time[-1] / .0001) - 2, .0001)
#         # time[-1] / .0001 - 2
        
#         print("force magnitudes", blade_force_magnitude[:20])
#         fig = plt.figure()
#         plt.plot(time, blade_force_magnitude)
#         plt.xlabel('Time (s)')
#         plt.ylabel('Blade Force Magnitude (Nm)')
#         plt.title(f'{wing_name} Blade Number {blade_number}')
#         plt.grid(True)
#         fig.savefig(folder_name + '/force_mag_' + str(wing_name) + "_" + str(blade_number) + '.png')
#         plt.show(block=False)

#         fig = plt.figure()
#         plt.plot(time, blade_velocity_magnitude)
#         plt.xlabel('Time (s)')
#         plt.ylabel('Blade Velocity Magnitude (m/s)')
#         plt.title(f'{wing_name} Blade Number {blade_number}')
#         plt.grid(True)
#         fig.savefig(folder_name + '/vel_mag_'+ str(wing_name) + "_" + str(blade_number) + '.png' )
#         plt.show()
        # Step 3: Create a 3D plot for each blade number
        # print("mean force x", np.mean(blade_force_z))
        # plt.figure()
        # plt.scatter(time, blade_force_z)
        # plt.xlabel('Time')
        # plt.ylabel('Blade Force Z')
        # plt.title(f'Blade Number {blade_number}')
        # plt.grid(True)
        # plt.show()

        # fig = plt.figure()
        # plt.plot(frequencies, np.abs(fft_result))
        # plt.xlabel('Frequency (Hz)')
        # plt.ylabel('Amplitude')
        # plt.title(f'{wing_name} FFT of Blade Force Z - Blade Number {blade_number}')
        # plt.grid(True)
        # fig.savefig(folder_name + '/fft_blade'+ wing_name + "_" + str(blade_number) +'.png')
        # plt.show()

        # Step 4: Plot the blade force vectors as arrows
        # ax.quiver(0, 0, 0, blade_force_x, blade_force_y, blade_force_z, label='Blade Force')
        # ax.set_xlabel('Blade Force X')
        # ax.set_ylabel('Blade Force Y')
        # ax.set_zlabel('Blade Force Z')
        # ax.set_title(f'Blade Number {blade_number}')

        # Step 5: Show the plot
        # plt.legend()
        # plt.show()

        #big plot
        # major_locator = ticker.MultipleLocator(base=.1)
        # minor_locator = ticker.MultipleLocator(base=0.05)
        # start_time = 3
        # end_time = 3.3
        # start_idx = np.argmax(time >= start_time)
        # end_idx = np.argmax(time >= end_time)
        # time_slice = time[start_idx:end_idx]
        # angle_slice = joint_LW_J_Pitch_deg[start_idx:end_idx]
        # fig = plt.figure()
        # plt.plot(time_slice, angle_slice, label='Angle Data (3s to 3.3s)')
        # plt.gca().xaxis.set_major_locator(major_locator)
        # plt.gca().xaxis.set_minor_locator(minor_locator)
        # plt.xlabel('Time (s)')
        # plt.ylabel('Angle (deg)')
        # plt.legend()
        # fig.savefig(folder_name + '/joint_angles_zoom.png')
        # plt.show()
        