import json
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

import pandas as pd




if len(sys.argv) < 2:
    print("Usage: python3 input_plot.py <input file location relative>")
    file_name = 'input_joint_data.csv'
    print("No folder path provided, pulling from default file: " + file_name)
else:
    file_name = sys.argv[1]

input_file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), file_name)
print("Reading from: " + input_file_path)

with open(input_file_path, 'r') as file:
    data = pd.read_csv(file)
     
    plt.figure(figsize=(10, 6))
    plt.plot(data['time'], data['input_torque'], label='Input Torque vs Time')
    plt.title('Input Torque vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Input Torque (Nm)')
    plt.grid(True)
    plt.show()
