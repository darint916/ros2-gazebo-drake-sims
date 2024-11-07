import json
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons
import numpy as np
import os

def param_to_xy(params: list):
    a, b, c, d, e = params
    x = np.linspace(0, 2, 1000000) # 3 seconds sim, sim runs at 0.00

    #Gamma const + amp con :Betatest 4
    y = a * ((1 - b) * np.sin(2 * np.pi * c * x + d) + (b * np.cos(4 * np.pi * c * x + e))) 

    return [x, y]

target_file = os.path.join(os.path.dirname(__file__), 'data/Beta_test__m_motor/data.json')

with open(target_file, 'r') as file:
    json_data = json.load(file)

top_results = sorted(json_data, key=lambda x: x['cost'], reverse=False)[:5] #lowest cost first, top 5
x_vals = []
y_vals = []
labels = []
# button_labels = []
for i, result in enumerate(top_results):
    x, y = param_to_xy(result['opt_params'])
    x_vals.append(x)
    y_vals.append(y)
    labels.append(f"Cost: {round(result['cost'],3)}")
    print(f"Cost: {round(result['cost'],3)}") 
    #now print parameters
    print(f"Optimized Parameters: {result['opt_params']}")
    # button_labels.append(f"Cost: {round(result['cost'],3)} ")

fig, ax = plt.subplots()
lines = []
for x, y, label in zip(x_vals, y_vals, labels):
    line, = ax.plot(x, y, label=label)
    lines.append(line)

ax.legend()
rax = plt.axes([0, 0.85, 0.1, 0.15])
check = CheckButtons(rax, labels, [True] * len(labels))

def toggle_visibility(label):
    index = labels.index(label)
    lines[index].set_visible(not lines[index].get_visible())
    plt.draw()

check.on_clicked(toggle_visibility)

plt.show()

    # plt.figure(figsize=(10,6))
    # plt.plot(x, y, label='Optimized Paramters')
    # plt.title('Optimized Parameters')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Voltage')
    # plt.grid(True)