from optimization.sdf_generate import generate_sdf
import os

FILE_PATH = os.path.abspath(__file__)
def sim_start(iteration:int):
    generate_sdf(os.path.join(FILE_PATH, 'data', 'iteration_{iteration}', 'iterative_wing.sdf'))
    
    # This function will start the simulation for the given iteration
    # It will return the fitness value of the simulation
    # The simulation will be started with the given parameters
    # The parameters will be read from the file and the simulation will be started
    # The fitness value will be read from the file and returned
    pass


iterations = 100
for i in range(iterations):
    