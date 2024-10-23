import json
import matplotlib.pyplot as plt
import numpy as np
import os

target_file = os.path.join(__file__, 'data/Beta_test_2_constraint_second_wave/data.json')

with open(target_file, 'r') as file:
    json_data = json.load(file)

top_results = sorted(json_data, key=lambda x: x['cost'], reverse=True)[:5] #lowest cost first, top 5

def plot_params_gamma_constraint(params: list):
    a, b, c, d, e = params
    x = np.linspace(0, 3, 1000) # 3 seconds sim, sim runs at 0.00
    