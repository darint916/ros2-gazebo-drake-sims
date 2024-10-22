from optimization.sdf_generate import generate_sdf
from optimization.cost_update import parse_data
from optimization.aero_properties import aero_properties
from utils.message import Message
import optimization.wing_classes as wing_classes
import scipy as sc
from scipy.optimize import OptimizeResult
import os
import shutil
import csv
import json
import subprocess
import time
import signal
import numpy as np
from unit_tests.test_wing_classes import check_triangle_inequalities
DIR_PATH = os.path.dirname(os.path.abspath(__file__))
json_config_path = os.path.join(DIR_PATH, 'data', 'input_config.json')

global population_counter
population_counter = 0
global iteration_counter
iteration_counter = 0

global folder_path
global counter
counter = 0

'''
Here we are optimizing for the wing input control,
V'' = -w * V
mainly diff evol
V = A * sin(wt + phi) + B * sin(2*wt + theta)
Params = [A, B, w, phi, theta]
'''
def write_to_json(data: dict, path: str) -> None:
    with open(path, 'r') as file:
        existing_data = json.load(file)
    existing_data.append(data)
    with open(path, 'w') as file:
        json.dump(existing_data, file, indent=4)

def top_start(iterations: int, title: str = "Beta_test_2_constraint_second_wave", popsize: int = 15):
    global folder_path
    folder_path = os.path.join(DIR_PATH, 'data', title)
    if not os.path.exists(folder_path):
        os.mkdir(folder_path)
    if not os.path.exists(os.path.join(folder_path, 'data.json')):
        with open(os.path.join(folder_path, 'data.json'), 'w') as file:
            json.dump([], file)
        
    Message.debug("TESTING: " + title, True)
    Message.info("folder path: " + folder_path)

    # '''BASE TEST 1'''
    # lower and upper bounds for parameters [A, B, w, phi, theta]
    # [V, V, Hz, rad, rad]
    # parameter_lower_bound = np.array([0, 0, 5, -np.pi, -np.pi])
    # parameter_upper_bound = np.array([50, 50, 60, np.pi, np.pi])
    # bounds = sc.optimize.Bounds(parameter_lower_bound, parameter_upper_bound)

    '''BASE TEST 2 (GAMMA PARAM)'''
    # lower and upper bounds for parameters [A, gamma, w, phi, theta]
    # [V, V, Hz, rad, rad]
    parameter_lower_bound = np.array([0, 0, 5, -np.pi, -np.pi])
    parameter_upper_bound = np.array([50, 1, 45, np.pi, np.pi])
    bounds = sc.optimize.Bounds(parameter_lower_bound, parameter_upper_bound)
    
    
    # products of A and opt_params gives a constraint on variables
    # in order, 
    
    # spar_0_angle - spar_1_angle >= 0
    # A = np.array([[0, 0, 1, 0, -1, 0, 0]  # represents linear constraints
    #               ])
    # linear_constraint_lower_bound = np.array([0])
    # linear_constraint_upper_bound = np.array([np.inf])
    # linear_constraints = sc.optimize.LinearConstraint(
    #     A, linear_constraint_lower_bound, linear_constraint_upper_bound)

    # initial_guess = np.array([7, 7, 25, 0, 0])
    # initial_guess = np.array([
    #     48.083774977833606,
    #     35.580658820604796,
    #     40.368480031318434,
    #     1.1037502606381693,
    #     1.006028981733762
    # ])
    initial_guess = np.array([30, 0, 25, 0, 0])
    sc.optimize.differential_evolution(sim_start, bounds, x0 = initial_guess,
                                       strategy='best1bin', maxiter=iterations, popsize=popsize, polish=False, callback=opt_callback)
    # This function will start the simulation for the given iteration
    # It will return the fitness value of the simulation
    # The simulation will be started with the given parameters
    # The parameters will be read from the file and the simulation will be started
    # The fitness value will be read from the file and returned

# Main Entrypoint of iteration
def sim_start(opt_params):
    # opt_params: wing_length, spar_0_length, spar_0_angle, spar_1_length, spar_1_angle, stroke_stiffness, pitch_stiffness
    # Bezier wing too hard to make
    with open(json_config_path, 'r') as json_file:
        config = json.load(json_file)
    # Message.info("\nSim iter start \n opt_params: 1. wave one amp; 2. wave two amp; 3. freq; 4. phase one; 5. phase two; \n " + str(opt_params)) #BETA TEST 1
    Message.info("\nSim iter start \n opt_params: 1. wave one amp; 2. wave two amp scale ; 3. freq; 4. phase one; 5. phase two; \n " + str(opt_params)) #BETA TEST 2
    config["voltage"]["waves"][0]["amplitude"] = opt_params[0]
    # config["voltage"]["waves"][1]["amplitude"] = opt_params[1] #BETA TEST 1
    config["voltage"]["waves"][1]["amplitude"] = opt_params[1] * opt_params[0] #BETA TEST 2
    config["voltage"]["frequency"] = opt_params[2]
    config["voltage"]["waves"][0]["phase"] = opt_params[3]
    config["voltage"]["waves"][1]["phase"] = opt_params[4]
    with open(json_config_path, 'w') as file:  # comment out to not update
        json.dump(config, file, indent=4)
    Message.info("config updated")
    sim_launch(config["sim_length"])
    Message.success("sim finished, parsing data")
    global counter
    counter += 1
    global iteration_counter
    iter_path = os.path.join(folder_path, f'iter_{iteration_counter}')
    if not os.path.exists(iter_path):
        os.mkdir(iter_path)
    global population_counter

    #copying input config files
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'input_config.json'),
    #             os.path.join(iter_path, f'config_{population_counter}.json'))
    population_counter += 1
    Message.debug("Simulation Generation Total: " + str(counter))

    cost = parse_data() * 2
    result_json = {
        "opt_params": opt_params.tolist(),
        "cost": cost
    }
    write_to_json(result_json, os.path.join(folder_path, 'data.json'))
    # Save config every iteration
    return cost #returns cost, * 2 cuz 2 wings
    # end of sim, save parameters and such

# copies generated data files to a new folder for each iteration


def opt_callback(intermediate_result: OptimizeResult) -> bool:
    print("callback")
    # nit = number iterations
    iter_path = os.path.join(folder_path, f'iter_{intermediate_result.nit}')
    global iteration_counter
    iteration_counter = intermediate_result.nit
    global population_counter
    population_counter = 0
    Message.debug("iter Path: ", iter_path)
    if not os.path.exists(iter_path):
        os.mkdir(iter_path)
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'input_config.json'), os.path.join(iter_path, 'input_config.json'))
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'aero.csv'), os.path.join(iter_path, 'aero.csv'))
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'data.csv'), os.path.join(iter_path, 'data.csv'))
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'processed.sdf'), os.path.join(iter_path, 'processed.sdf'))
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'input_joint_data.csv'), os.path.join(iter_path, 'input_joint_data.csv'))
    opt_res = os.path.join(iter_path, 'opt_res.json')
    Message.info("opt_res path: ", opt_res)

    # Save the intermediate result to a json file
    attributes = dir(intermediate_result)
    result_data = {}
    filtered_attributes = [attr for attr in attributes if not attr.startswith(
        '_') and not callable(getattr(intermediate_result, attr))]
    for attr in filtered_attributes:
        value = getattr(intermediate_result, attr)
        result_data[attr] = value

    with open(opt_res, 'w') as file:
        json.dump(result_data, file, cls=NumpyEncoder, indent=4)
    return False

# Launches IGN Gaz, and handles only the GRASS SIMULATION
# Note that ign gui does not get killed by SIGTERM (or any signal), requires PID kill process
def sim_launch(sim_time: float) -> None:
    launch_path = os.path.join(DIR_PATH, '..', 'launch', 'hardrobot_launch.py')
    ros2_process = subprocess.Popen(
        ['ros2', 'launch', launch_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    # Path mirrored from opt_launch.py
    kill_file_path = os.path.join(DIR_PATH, '_kill_me.txt')
    try:
        time.sleep(sim_time)
        while (ros2_process.poll() is None):  # most scuffed polling process
            time.sleep(.5)
            if os.path.isfile(kill_file_path):
                Message.debug("kill signal received")
                break
    # check processes with `ps aux | grep 'ign|gz', '^(?!.*signal).*ign.*|.*gz.*'
    finally:
        if os.path.isfile(kill_file_path):
            os.remove(kill_file_path)
        ros2_process.send_signal(signal.SIGINT)
        ros2_process.wait()
        for line in ros2_process.stdout:  # terminal output for sim
            print(line.decode(), end='')
        ros2_process.terminate()
    try:  # Neg lookahead does ot work for pkill, manual pid filter
        # subprocess.run("pkill -f '^(?!.*signal).*ign.*|.*gz.*'", shell=True, check=True)
        proc = subprocess.run(['pgrep', '-af', 'ign|gz'],
                              text=True, capture_output=True)
        processes = proc.stdout.splitlines()
        targeted_processes = [p.split()[0]
                              for p in processes if 'signal' not in p]
        for pid in targeted_processes:
            subprocess.run(['kill', '-9', pid])
        Message.success("ignition kill success")
    except subprocess.CalledProcessError as e:
        Message.error("termination fail:", e)


class NumpyEncoder(json.JSONEncoder):
    """ Custom encoder for numpy data types """

    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


if __name__ == '__main__':
    top_start(250, popsize=5)
    # generate_sdf()
    # sim_launch()
