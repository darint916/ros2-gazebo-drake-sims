import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Step 1: Read the CSV file skipping the first 4 rows
csv_file = 'aero.csv'
df = pd.read_csv(csv_file, skiprows=range(1,2))
print(df.columns)
# Step 2: Create separate plots for each blade number
unique_wing_names = df['wing_name'].unique()
unique_blade_numbers = df['blade_number'].unique()

for wing_name in unique_wing_names:
    print("wing name", wing_name)
    for blade_number in unique_blade_numbers:
        blade_data = df[(df['wing_name'] == wing_name) & (df['blade_number'] == blade_number)]
        blade_force_x = blade_data['blade_force_x']
        blade_force_y = blade_data['blade_force_y']
        blade_force_z = blade_data['blade_force_z'].to_numpy()
        blade_force_magnitude = blade_force_x ** 2 + blade_force_y ** 2 + blade_force_z ** 2
        blade_force_magnitude = np.sqrt(blade_force_magnitude).to_numpy()
        time = blade_data['time'].to_numpy()
        # print("time first 10 rows", time[:10])
        # print("asdf", time[-1] )
        # print("asdf", len(time))
        fft_result = np.fft.fft(blade_force_z)
        # fft_result = np.fft.fft(blade_force_magnitude)
        print("fft res len", len(fft_result))
        # print("z force len", len(blade_force_z))
        print("time len", int(time[-1] / .0001) - 2)
        frequencies = np.fft.fftfreq(int(time[-1] / .0001) - 2, .0001)
        # time[-1] / .0001 - 2
        
        print("force magnitudes", blade_force_magnitude[:20])
        plt.figure()
        plt.plot(time, blade_force_magnitude)
        plt.xlabel('Time (s)')
        plt.ylabel('Blade Force Magnitude (N)')
        plt.title(f'Blade Number {blade_number}')
        plt.grid(True)
        plt.show()


        # Step 3: Create a 3D plot for each blade number
        # print("mean force x", np.mean(blade_force_z))
        # plt.figure()
        # plt.scatter(time, blade_force_z)
        # plt.xlabel('Time')
        # plt.ylabel('Blade Force Z')
        # plt.title(f'Blade Number {blade_number}')
        # plt.grid(True)
        # plt.show()

        plt.figure()
        plt.plot(frequencies, np.abs(fft_result))
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Amplitude')
        plt.title(f'FFT of Blade Force Z - Blade Number {blade_number}')
        plt.grid(True)
        plt.show()

        # Step 4: Plot the blade force vectors as arrows
        # ax.quiver(0, 0, 0, blade_force_x, blade_force_y, blade_force_z, label='Blade Force')
        # ax.set_xlabel('Blade Force X')
        # ax.set_ylabel('Blade Force Y')
        # ax.set_zlabel('Blade Force Z')
        # ax.set_title(f'Blade Number {blade_number}')

        # Step 5: Show the plot
        # plt.legend()
        # plt.show()