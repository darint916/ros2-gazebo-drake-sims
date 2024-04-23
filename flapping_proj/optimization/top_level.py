from optimization.sdf_generate import generate_sdf
from optimization.cost_update import parse_data
from optimization.aero_properties import aero_properties
import inertial_properties
import scipy as sc
from scipy.optimize import OptimizeResult
import os
import shutil
import csv
import json
import subprocess
import time
import signal
DIR_PATH = os.path.dirname(os.path.abspath(__file__))
json_config_path = os.path.join(DIR_PATH,'data','config.json')
with open(json_config_path, 'r') as json_file:
    config = json.load(json_file)
# iterator_counter = 0
global folder_path
def top_start(iterations:int,title:str = "beta_test"):
    folder_path = os.path.join(DIR_PATH, 'data', title)
    if not os.path.exists(folder_path):
        os.mkdir(folder_path)
    
    # sc.optimize.differential_evolution(sim_cost, 
    # for i in range(iterations):
    #     sim_start(i, folder_path)
    
    # This function will start the simulation for the given iteration
    # It will return the fitness value of the simulation
    # The simulation will be started with the given parameters
    # The parameters will be read from the file and the simulation will be started
    # The fitness value will be read from the file and returned

def pitch_stiffness_calc(length:float, width:float) ->float: #lw of hinge
    youngs_modulus = 14.6 #GPa
    wing_thickness = 12e-6 #m 
    return youngs_modulus * width * wing_thickness**3 / 4.0 / (length**2) #Nm/rad
     


def sim_start(opt_params):
    #opt_params: y0, z0, y1, z1,.. y3, z3, stroke_stiffness, 
    b_curve = inertial_properties.BezierCurve(opt_params[0], opt_params[1], opt_params[2], opt_params[3], opt_params[4], opt_params[5], opt_params[6], opt_params[7])
    wing_inertia = inertial_properties.wing_I(b_curve)
    wing_mass = inertial_properties.wing_mass(b_curve)
    config["wing"]["inertia"]["ixx"] = wing_inertia[0]
    config["wing"]["inertia"]["iyy"] = wing_inertia[1]
    config["wing"]["inertia"]["izz"] = wing_inertia[2]
    config["wing"]["inertia"]["ixy"] = wing_inertia[3]
    config["wing"]["inertia"]["ixz"] = wing_inertia[4]
    config["wing"]["inertia"]["iyz"] = wing_inertia[5]
    config["wing"]["mass"] = wing_mass
    config["stroke_joint"]["spring_stiffness"] = opt_params[8]
    chord_cp, spar_cp, blade_areas = aero_properties(b_curve, config["blade_count"])
    #length = |z0|, width = y3 - y0
    config["pitch_joint"]["spring_stiffness"] = pitch_stiffness_calc(abs(opt_params[1]), opt_params[6] - opt_params[0])
    

    generate_sdf(chord_cp=chord_cp, spar_cp=spar_cp, blade_area=blade_areas) #write to default location: (data/processed.sdf)
    sim_launch
    #end of sim, save parameters and such

def opt_callback(intermediate_result: OptimizeResult):
    iter_path = os.path.join(folder_path, f'iter_{intermediate_result.nit}')
    if not os.path.exists(iter_path):
        os.mkdir(iter_path)
    shutil.copy(os.path.join(DIR_PATH, 'data', 'config.json'), os.path.join(iter_path, 'config.json'))
    shutil.copy(os.path.join(DIR_PATH, 'data', 'aero.csv'), os.path.join(iter_path, 'aero.csv'))
    shutil.copy(os.path.join(DIR_PATH, 'data', 'data.csv'), os.path.join(iter_path, 'aero.csv'))
    shutil.copy(os.path.join(DIR_PATH, 'data', 'processed.sdf'), os.path.join(iter_path, 'processed.sdf'))
    opt_res = os.path.join(iter_path, 'opt_res.csv')
    with open(opt_res, mode='a') as file:
        writer = csv.writer(file)
        writer.writerow(intermediate_result.tolist())
    return False 


def sim_launch(): #Note that ign gui does not get killed by SIGTERM (or any signal), requires PID kill process
    launch_path = os.path.join(DIR_PATH, '..', 'launch', 'opt_launch.py')
    ros2_process = subprocess.Popen(['ros2', 'launch', launch_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    sim_time = config["sim_length"]
    kill_file_path = os.path.join(DIR_PATH, '_kill_me.txt') #Path mirrored from opt_launch.py
    try:
        time.sleep(sim_time)
        while(ros2_process.poll() is None): #most scuffed polling process
            time.sleep(.5)
            if os.path.isfile(kill_file_path):
                print("kill signal received")
                break
    finally: #check processes with `ps aux | grep 'ign|gz', '^(?!.*signal).*ign.*|.*gz.*'
        if os.path.isfile(kill_file_path):
            os.remove(kill_file_path)
        ros2_process.send_signal(signal.SIGINT)
        print("kill wait")
        # time.sleep(1)
        # ros2_process.send_signal(signal.SIGINT)
        ros2_process.wait()  
        for line in ros2_process.stdout: #terminal output for sim
            print(line.decode(), end='')
        print("ros2 done")
        ros2_process.terminate()
    try: # Neg lookahead does ot work for pkill, manual pid filter
        # subprocess.run("pkill -f '^(?!.*signal).*ign.*|.*gz.*'", shell=True, check=True) 
        proc = subprocess.run(['pgrep', '-af', 'ign|gz'], text=True, capture_output=True)
        processes = proc.stdout.splitlines()
        targeted_processes = [p.split()[0] for p in processes if 'signal' not in p]
        for pid in targeted_processes:
            subprocess.run(['kill', '-9', pid])
    except subprocess.CalledProcessError as e:
        print("termination fail:", e)
if __name__ == '__main__':
    # top_start(1)
    # generate_sdf()
    sim_launch()