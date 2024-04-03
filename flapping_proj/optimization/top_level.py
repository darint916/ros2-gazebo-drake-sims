from optimization.sdf_generate import generate_sdf
import os
import shutil
DIR_PATH = os.path.dirname(os.path.abspath(__file__))

def top_start(iterations:int,title:str = "beta_test"):
    folder_path = os.path.join(DIR_PATH, 'data', title)
    if not os.path.exists(folder_path):
        os.mkdir(folder_path)
    
    for i in range(iterations):
        sim_start(i, folder_path)
    
    # This function will start the simulation for the given iteration
    # It will return the fitness value of the simulation
    # The simulation will be started with the given parameters
    # The parameters will be read from the file and the simulation will be started
    # The fitness value will be read from the file and returned
    pass

def sim_start(iteration:int, path:str):
    iter_path = os.path.join(path, f'iter_{iteration}')
    if not os.path.exists(iter_path):
        os.mkdir(iter_path)
    # Load the JSON configuration
    
    generate_sdf() #write to default location: (data/processed.sdf)
    
    #end of sim, save parameters and such
    # shutil.copy(os.path.join(FILE_PATH, 'data', 'config.json'), os.path.join(iter_path, 'config.json'))
    # shutil.copy(os.path.join(FILE_PATH, 'data', 'aero.csv'), os.path.join(iter_path, 'aero.csv'))
    # shutil.copy(os.path.join(FILE_PATH, 'data', 'data.csv'), os.path.join(iter_path, 'aero.csv'))

if __name__ == '__main__':
    top_start(1)