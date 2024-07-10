from optimization.sdf_generate import generate_sdf
from optimization.cost_update import parse_data
from optimization.aero_properties import aero_properties
from utils.message import Message
import wing_classes
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
json_config_path = os.path.join(DIR_PATH,'data','config.json')

global population_counter
population_counter = 0
global iteration_counter 
iteration_counter = 0

global folder_path
global counter
counter = 0
def top_start(iterations:int,title:str = "beta_test", popsize:int = 15):
    global folder_path
    folder_path = os.path.join(DIR_PATH, 'data', title)
    if not os.path.exists(folder_path):
        os.mkdir(folder_path)
    Message.debug("TESTING: " + title, True)
    Message.info("folder path: " + folder_path)
    
    #lower and upper bounds for [y0, z0, y1, z1, y2, z2, k_phi]
    # parameter_lower_bound = np.array([1, -3, 1, -np.inf, 1, -np.inf, .01]) / 1000 #[mm, mm, mm, mm, mm, mm, mNm]
    #bounds converted to si by the /1000
    parameter_lower_bound = np.array([7, -3, 1, -200, 1, -2, 1]) / 1000 #[mm, mm, mm, mm, mm, mm, mNm] 
    parameter_upper_bound = np.array([30, -1, 150, -1, 150, 0, 100]) / 1000 #[mm, mm, mm, mm, mm, mm, mNm]
    #position coordinates describing wing: (0, y0, z0), (0, y1, z1), (0, y2, z2)
    #y0 is the horizontal location nearest to the root. 
    #z0 is the vertical location of the film near the root
    #y1 and z1 gives the tip of the triangle
    #y2 is the length of the wing
    #realistically z2 should probably be constrained to 0
    #k_phi is the stiffness about the flapping axis

    bounds = sc.optimize.Bounds(parameter_lower_bound, parameter_upper_bound)
    
    #products of A and opt_params gives a constraint on variables
    #in order, 
    #y1 - y0 >= 0.001
    #y2 - y0 >= 0.001
    #y2 - y1 >= 0.001
    #z0 - z1 >= 0
    A = np.array([[-1, 0, 1, 0, 0, 0, 0], #represents linear constraints
                  [-1, 0, 0, 0, 1, 0, 0],
                  [ 0, 0,-1, 0, 1, 0, 0],
                  [ 0, 1, 0,-1, 0, 0, 0]])
    constraint_lower_bound = np.array([0.001, 0.001, 0.001, 0])
    constraint_upper_bound = np.array([np.inf, np.inf, np.inf, np.inf])
    constraints = sc.optimize.LinearConstraint(A, constraint_lower_bound, constraint_upper_bound)

    sc.optimize.differential_evolution(sim_start, bounds, constraints = constraints,
            strategy='best1bin', maxiter=iterations, popsize = popsize, polish=False, callback=opt_callback)
    # This function will start the simulation for the given iteration
    # It will return the fitness value of the simulation
    # The simulation will be started with the given parameters
    # The parameters will be read from the file and the simulation will be started
    # The fitness value will be read from the file and returned
    

def pitch_stiffness_calc(length:float, width:float) ->float: #lw of hinge
    youngs_modulus = 14.6 #GPa
    wing_thickness = 12e-6 #m 
    return youngs_modulus * width * wing_thickness**3 / 4.0 / (length**2) #Nm/rad
    
#Main Entrypoint of iteration
def sim_start(opt_params):
    #opt_params: y0, z0, y1, z1,.. y2, z2, stroke_stiffness, 
    #Bezier wing too hard to make
    with open(json_config_path, 'r') as json_file:
        config = json.load(json_file)
    Message.data("sim iter start \n opt_params: " + str(opt_params))
    tri_wing = wing_classes.TriWing(opt_params[0], opt_params[1], opt_params[2], opt_params[3], opt_params[4], opt_params[5])
    config["wing"]["inertia"]["ixx"] = tri_wing.I[1]
    config["wing"]["inertia"]["iyy"] = tri_wing.I[0]
    config["wing"]["inertia"]["izz"] = tri_wing.I[2]
    config["wing"]["inertia"]["ixy"] = tri_wing.I[3]
    config["wing"]["inertia"]["ixz"] = tri_wing.I[5]
    config["wing"]["inertia"]["iyz"] = tri_wing.I[4]
    check_triangle_inequalities(tri_wing.I[0], tri_wing.I[1], tri_wing.I[2])
    config["wing"]["mass"] = tri_wing.mass
    config["stroke_joint"]["spring_stiffness"] = opt_params[6]
    config["wing"]["com"] = [tri_wing.com[0], 0, tri_wing.com[1], 0, 0, 0]
    Message.debug("config inertia:")
    Message.debug(config["wing"]["inertia"])
    chord_cp, spar_cp, blade_areas = aero_properties(tri_wing, config["blade_count"])
    #length = |z0|, width = y3 - y0
    config["pitch_joint"]["spring_stiffness"] = pitch_stiffness_calc(abs(opt_params[1]), opt_params[4] - opt_params[0])
    with open(json_config_path, 'w') as file: #comment out to not update
        json.dump(config, file, indent=4)
    Message.info("config updated")
    generate_sdf(chord_cps=chord_cp, spar_cps=spar_cp, blade_area=blade_areas) #write to default location: (data/processed.sdf)
    Message.success("sdf generated, now launching sim")
    sim_launch(config["sim_length"])
    Message.success("sim finished, parsing data")
    global counter
    counter += 1 
    global iteration_counter
    iter_path = os.path.join(folder_path, f'iter_{iteration_counter}')
    if not os.path.exists(iter_path):
        os.mkdir(iter_path)
    global population_counter
    shutil.copy(os.path.join(DIR_PATH, 'data', 'config.json'), os.path.join(iter_path, f'config_{population_counter}.json'))
    population_counter += 1
    Message.debug("Simulation Generation Total: " + str(counter))

    #Save config every iteration
    return parse_data()
    #end of sim, save parameters and such

#copies generated data files to a new folder for each iteration
def opt_callback(intermediate_result: OptimizeResult) -> bool:
    print("callback")
    iter_path = os.path.join(folder_path, f'iter_{intermediate_result.nit}') #nit = number iterations
    global iteration_counter
    iteration_counter = intermediate_result.nit
    global population_counter
    population_counter = 0
    Message.debug("iter Path: ", iter_path)
    if not os.path.exists(iter_path):
        os.mkdir(iter_path)
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'config.json'), os.path.join(iter_path, 'config.json'))
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'aero.csv'), os.path.join(iter_path, 'aero.csv'))
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'data.csv'), os.path.join(iter_path, 'data.csv'))
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'processed.sdf'), os.path.join(iter_path, 'processed.sdf'))
    # shutil.copy(os.path.join(DIR_PATH, 'data', 'input_joint_data.csv'), os.path.join(iter_path, 'input_joint_data.csv'))
    opt_res = os.path.join(iter_path, 'opt_res.json')
    Message.info("opt_res path: ", opt_res)

    attributes = dir(intermediate_result)
    result_data = {}
    filtered_attributes = [attr for attr in attributes if not attr.startswith('_') and not callable(getattr(intermediate_result, attr))]
    for attr in filtered_attributes:
        value = getattr(intermediate_result, attr)
        result_data[attr] = value

    with open(opt_res, 'w') as file:
        json.dump(result_data, file, cls=NumpyEncoder, indent=4)
    return False 

#Launches IGN Gaz, and handles only the GRASS SIMULATION
def sim_launch(sim_time: float) -> None: #Note that ign gui does not get killed by SIGTERM (or any signal), requires PID kill process
    launch_path = os.path.join(DIR_PATH, '..', 'launch', 'opt_launch.py')
    ros2_process = subprocess.Popen(['ros2', 'launch', launch_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    kill_file_path = os.path.join(DIR_PATH, '_kill_me.txt') #Path mirrored from opt_launch.py
    try:
        time.sleep(sim_time)
        while(ros2_process.poll() is None): #most scuffed polling process
            time.sleep(.5)
            if os.path.isfile(kill_file_path):
                Message.debug("kill signal received")
                break
    finally: #check processes with `ps aux | grep 'ign|gz', '^(?!.*signal).*ign.*|.*gz.*'
        if os.path.isfile(kill_file_path):
            os.remove(kill_file_path)
        ros2_process.send_signal(signal.SIGINT)
        ros2_process.wait()  
        for line in ros2_process.stdout: #terminal output for sim
            print(line.decode(), end='')
        ros2_process.terminate()
    try: # Neg lookahead does ot work for pkill, manual pid filter
        # subprocess.run("pkill -f '^(?!.*signal).*ign.*|.*gz.*'", shell=True, check=True) 
        proc = subprocess.run(['pgrep', '-af', 'ign|gz'], text=True, capture_output=True)
        processes = proc.stdout.splitlines()
        targeted_processes = [p.split()[0] for p in processes if 'signal' not in p]
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
    top_start(350, popsize=7)
    # generate_sdf()
    # sim_launch()