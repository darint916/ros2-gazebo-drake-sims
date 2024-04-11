#cost script needs to take your simulator output, 

#parse aero.csv
#average the lift force for the last few cycles
#get motor output angle for that time period
#use it to figure out the torque supplied based on the input voltage

# u = r_a * i_a + k_a * omega, k_a * i_a = torque where u is input voltage, r_a is motor resistance, i_a is motor current, k_a is the torque constant of the motor, and omega is the rotational velocity of the motor
# torque = (u - k_a * omega) * k_a / r_a
#r_a = motor_resistance 
#k_a = motor_torque_constant
# power is just torque * omega
# tbh just apply a sinusoidal torque for now, and can calculate power from input and measured omega
# for instantanious power, need the root mean square average over the last .2 seconds, for lift need the straight average over the last .2 seconds

#data parse
import os
curr_dir = os.path.dirname(os.path.abspath(__file__))
def parse_data(time_start, time_end, iteration:int):
    with open(os.path.join(curr_dir, 'data', f'iter_{iteration}', 'aero.csv'), 'r') as file:
        #read files from time_start to time_end
        data = file.readlines()
        aero = data[time_start:time_end]
        for 
