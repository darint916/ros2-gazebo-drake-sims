#cost script needs to take your simulator output, 

#parse aero.csv
#average the lift force for the last few cycles
#get motor output angle for that time period
#use it to figure out the torque supplied based on the input voltage

# u = r_a * i_a + k_a * omega, k_a * i_a = torque where u is input voltage, r_a is motor resistance, i_a is motor current, k_a is the torque constant of the motor, and omega is the rotational velocity of the motor
# torque = (u - k_a * omega) * k_a / r_a
# u = given list of motor voltage, k_a is motor torque
#r_a = motor_resistance 
#k_a = motor_torque_constant
# power is just torque * omega
# tbh just apply a sinusoidal torque for now, and can calculate power from input and measured omega
# for instantanious power, need the root mean square average over the last .2 seconds, for lift need the straight average over the last .2 seconds

#data parse
import os
from utils.message import Message
import numpy as np
import pandas as pd
import json
curr_dir = os.path.dirname(os.path.abspath(__file__))
json_config_path = os.path.join(curr_dir,'data','config.json')

#returns float of the current iteration cost function evaluation
def parse_data() -> float: 
    with open(json_config_path, 'r') as json_file:
        config = json.load(json_file)
    flap_freq = config["inputs"]["frequency"] #float Hz
    wing_mass = config["wing"]["mass"] #from config, float kg
    interest_duration = 4 / flap_freq #duration of time steps to evaluate for
    t_start = config["sim_length"] / 2
    dataname = open(os.path.join(curr_dir, 'data', 'data.csv'), 'r')
    aeroname = open(os.path.join(curr_dir, 'data', 'aero.csv'), 'r')
    #read files from time_start to time_end

    t_end = t_start + interest_duration

    data = pd.read_csv(dataname)
    aero = pd.read_csv(aeroname)

    data_interval = data[(data.time >= t_start) & (data.time <= t_end)]
    aero_interval = aero[(aero.time >= t_start) & (aero.time <= t_end)]

    aero_length = len(aero_interval.time)

    num_blades = np.amax(aero_interval.blade_number) + 1
    lift_force = np.zeros(int(aero_length / num_blades))
    i = 0 #aero_interval index
    j = 0 #lift force index
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
    #instant power = angular velocity about stroke axis * torque applied on stroke axis
    instant_power = data_interval.time * data_interval.position_z #needs to be changed to motor torque and stroke velocity
    power_rms = np.sqrt(np.sum(instant_power ** 2) / len(instant_power))

    lift_avg = float(lift_avg)
    power_rms = float(power_rms)
    wing_mass = float(wing_mass)

    Message.data("power_rms printing: "+ str(power_rms))
    Message.data("lift_avg printing: "+ str(lift_avg))
    Message.data("wing_mass printing: " + str(wing_mass))

    # if lift_avg == 0 or power_rms == 0 or wing_mass == 0:
    #     return 
    return -(lift_avg / power_rms / wing_mass)

if __name__ == "__main__":
    print(parse_data())