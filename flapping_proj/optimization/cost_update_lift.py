# cost script needs to take your simulator output,

# parse aero.csv
# average the lift force for the last few cycles
# get motor output angle for that time period
# use it to figure out the torque supplied based on the input voltage

# u = r_a * i_a + k_a * omega, k_a * i_a = torque where u is input voltage, r_a is motor resistance, i_a is motor current, k_a is the torque constant of the motor, and omega is the rotational velocity of the motor
# torque = (u - k_a * omega) * k_a / r_a
# u = given list of motor voltage, k_a is motor torque
# r_a = motor_resistance
# k_a = motor_torque_constant
# power is just torque * omega
# tbh just apply a sinusoidal torque for now, and can calculate power from input and measured omega
# for instantanious power, need the root mean square average over the last .2 seconds, for lift need the straight average over the last .2 seconds

# data parse
import os
from utils.message import Message
import numpy as np
import pandas as pd
import json
curr_dir = os.path.dirname(os.path.abspath(__file__))
#NOTE THIS USES INPUT CONFIG for freq
json_config_path = os.path.join(curr_dir, 'data', 'input_config.json')

# returns float of the current iteration cost function evaluation

def parse_data(flap_freq: float, sim_time: float) -> float:
    interest_duration = 4 / flap_freq  # duration of time steps to evaluate for
    t_start = sim_time / 2
    aeroname = open(os.path.join(curr_dir, 'data', 'aero.csv'), 'r')
    # read files from time_start to time_end
    
    t_end = t_start + interest_duration
    aero = pd.read_csv(aeroname)
    aero_interval = aero[(aero.time >= t_start) & (aero.time <= t_end)]
    aero_length = len(aero_interval.time)

    lift_force = np.zeros(len(np.unique(aero_interval.time)))
    i = 0  # aero_interval index
    j = 0  # lift force index
    time_prev = aero_interval.time.iloc[0]
    while (i < aero_length):
        if aero_interval.time.iloc[i] == time_prev:
            pass
        else:
            time_prev = aero_interval.time.iloc[i]
            j += 1
        lift_force[j] += aero_interval.blade_force_z.iloc[i]
        i += 1
    lift_avg = np.average(lift_force)
    print("lift avg:", lift_avg)

    lift_avg = float(lift_avg)
    # Constant sources:
    # power_rms: 2 * 8.8 / 6: rms power consumed assuming the motor is at stall at all times (6V / 2 8.8 ohm)
    # lift_avg: 0.1470997807: target lift of 15g/wing over the average cycle
    # This should normalize power to 1 and lift to 1000
    # cost = - lift_avg / 0.1470997807 * 10 + power_rms * 2 * 8.8 / 6
    cost = -lift_avg #for lift only

    Message.data("lift_avg printing: " + str(lift_avg))
    Message.data("cost total: " + str(cost))

    return cost

def get_min_max_joint_angles():
    with open(json_config_path, 'r') as json_file:
        config = json.load(json_file)

    curr_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(curr_dir, 'data', 'data.csv')
    data = pd.read_csv(data_path)

    flap_freq = config["voltage"]["frequency"]  # float Hz
    interest_duration = 4 / flap_freq  # duration of time steps to evaluate for
    t_start = config["sim_length"] / 2
    t_end = t_start + interest_duration

    data_interval = data[(data.time >= t_start) & (data.time <= t_end)]

    stroke_joint_1 = np.degrees(data_interval['stroke_joint_1'].to_numpy())
    stroke_joint_2 = np.degrees(data_interval['stroke_joint_2'].to_numpy())
    pitch_joint_1 =  np.degrees(data_interval['pitch_joint_1'].to_numpy())
    pitch_joint_2 =  np.degrees(data_interval['pitch_joint_2'].to_numpy())
    return [min(stroke_joint_1), max(stroke_joint_1), min(stroke_joint_2), max(stroke_joint_2), min(pitch_joint_1), max(pitch_joint_1), min(pitch_joint_2), max(pitch_joint_2)]
    

if __name__ == "__main__":
    parse_data(1, 1)
