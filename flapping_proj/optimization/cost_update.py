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
json_config_path = os.path.join(curr_dir, 'data', 'config.json')

# returns float of the current iteration cost function evaluation


def parse_data() -> float:
    with open(json_config_path, 'r') as json_file:
        config = json.load(json_file)
    flap_freq = config["inputs"]["frequency"]  # float Hz
    wing_mass = config["wing"]["mass"]  # from config, float kg
    # from config, float Nm/A
    torque_constant = config["inputs"]["motor_torque_constant"]
    # from config, float
    motor_gear_ratio = config["inputs"]["motor_gear_ratio"]
    # from config, float
    external_gear_ratio = config["inputs"]["external_gear_ratio"]
    # from config, float
    reduction_efficiency = config["inputs"]["reduction_efficiency"]
    interest_duration = 4 / flap_freq  # duration of time steps to evaluate for
    t_start = config["sim_length"] / 2
    dataname = open(os.path.join(curr_dir, 'data', 'data.csv'), 'r')
    aeroname = open(os.path.join(curr_dir, 'data', 'aero.csv'), 'r')
    input_joint_name = open(os.path.join(
        curr_dir, 'data', 'input_joint_data.csv'), 'r')
    # read files from time_start to time_end

    t_end = t_start + interest_duration

    data = pd.read_csv(dataname)
    aero = pd.read_csv(aeroname)
    input_joint = pd.read_csv(input_joint_name)

    data_interval = data[(data.time >= t_start) & (data.time <= t_end)]
    aero_interval = aero[(aero.time >= t_start) & (aero.time <= t_end)]
    input_joint_interval = input_joint[(
        input_joint.time >= t_start) & (input_joint.time <= t_end)]
    aero_length = len(aero_interval.time)

    num_blades = np.amax(aero_interval.blade_number) + 1
    lift_force = np.zeros(int(aero_length / num_blades))
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
    # V*I, v = input sin wave, I = output torque / motor torque const k_a
    effective_reduction = motor_gear_ratio * \
        external_gear_ratio * reduction_efficiency
    voltage = np.sin(data_interval.time * flap_freq * np.pi * 2)
    current = input_joint_interval.input_torque / \
        torque_constant / effective_reduction
    instant_power = voltage * current
    # instant power = angular velocity about stroke axis * torque applied on stroke axis
    # instant_power = data_interval.stroke_joint_velocity * input_joint_interval.stroke_joint_torque #needs to be changed to motor torque and stroke velocity
    power_rms = np.sqrt(np.sum(instant_power ** 2) / len(instant_power))

    # Message.debug("current printing: "+ str(current))
    # Message.debug("voltage printing: "+ str(voltage))
    # Message.debug("instant power printing: "+ str(instant_power))
    # Message.debug("power_rms printing: "+ str(power_rms))
    # Message.debug("lift_avg printing: "+ str(lift_avg))
    # Message.debug("wing_mass printing: " + str(wing_mass))

    lift_avg = float(lift_avg)
    power_rms = float(power_rms)
    wing_mass = float(wing_mass)
    # Constant sources:
    # power_rms: 2 * 8.8 / 6: rms power consumed assuming the motor is at stall at all times (6V / 2 8.8 ohm)
    # lift_avg: 0.1470997807 * 1000: target lift of 15g/wing over the average cycle * 1000 (lift matters a ton)
    # This should normalize power to 1 and lift to 1000
    cost = - lift_avg / 0.1470997807 * 1000

    Message.data("power_rms printing: " + str(power_rms))
    Message.data("lift_avg printing: " + str(lift_avg))
    Message.data("wing_mass printing: " + str(wing_mass))
    Message.data("cost: " + str(cost))

    
    return cost


if __name__ == "__main__":
    print(parse_data())
