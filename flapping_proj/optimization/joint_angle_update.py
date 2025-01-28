import os
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib.ticker as ticker
import shutil
import numpy as np
import sys
import math


def get_min_max_joint_angles():
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(curr_dir, 'data', 'data.csv')
    data = pd.read_csv(data_path)
    time = data['time'].to_numpy()

    pitch_joint_1 =  np.degrees(data['pitch_joint_1'].to_numpy())
    pitch_joint_2 =  np.degrees(data['pitch_joint_2'].to_numpy())
    stroke_joint_1 = np.degrees(data['stroke_joint_1'].to_numpy())
    stroke_joint_2 = np.degrees(data['stroke_joint_2'].to_numpy())

    