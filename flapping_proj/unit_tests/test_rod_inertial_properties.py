import unittest
import numpy as np
from optimization.inertial_properties import *
from utils.message import Message

# finds if two values are almost equal within a range based on
# a range defined by error_percent (in decimal form) times a
def almost_equal(a, b, error_percent):
    return abs(a - b) <= a*error_percent

def line_length(coords):
    return np.sqrt((coords[2] - coords[0])**2 + (coords[3] - coords[1])**2)

class TestSlenderRodInertialproperties(unittest.TestCase):
    error_tol = 0.001 #allowed percent error as a decimal
    rod_density = 1000 #kg/m**3
    rod_diameter = 0.01 #m
    rod_linear_density = rod_density * np.pi * rod_diameter**2 / 4

    #sets of coordinates of form [y0, z0, y1, z1]
    test_coordinates = [[-1,  0, 1, 0],
                        [ 0, -1, 0, 1],
                        [ 0,  0, 1, 0],
                        [ 0,  0, 0, 1],
                        [-1, -1, 1, 1],
                        [ 0,  0, 1, 1],
                        [-1, -2, 1, 2],
                        [-1,  1, 1, 1],
                        [ 1, -1, 1, 1]]
    
    truth_mass = np.zeros(len(test_coordinates))
    for i in range(len(test_coordinates)):
        coords = test_coordinates[i]
        truth_mass[i] = rod_linear_density * line_length(coords)

    def test_rod_mass(self):
        # Message.warning("Checking Rod Mass Calculation", bold=True)
        for i in range(len(self.test_coordinates)):
            coords = self.test_coordinates[i]
            calc_mass = line_m(coords[0], coords[1], coords[2], coords[3], diameter = self.rod_diameter, density = self.rod_density)

            self.assertTrue(almost_equal(self.truth_mass[i], calc_mass, self.error_tol), 
                            f"Mass case failed: {coords}. Expected {self.truth_mass[i]} but output was {calc_mass}")

            # Message.info(f"Analysing rod with endpoints (y, z) at ({coords[0]}, {coords[1]}) and ({coords[2]}, {coords[3]})")
            # if almost_equal(self.truth_mass[i], calc_mass, self.error_tol):
            #     Message.success("Passed Case")
            # else:
            #     Message.error("Failed case")
            
            # Message.debug(f"Truth Mass:      {self.truth_mass[i]}")
            # Message.debug(f"Calculated Mass: {calc_mass}")
            # print("")

    def test_rod_com(self):
        # Message.warning("Checking Center of Mass Calculations", bold=True)
        #center of mass truths
        truth_com = np.array([[  0,   0],
                              [  0,   0],
                              [0.5,   0],
                              [  0, 0.5],
                              [  0,   0],
                              [0.5, 0.5],
                              [  0,   0],
                              [  0,   1],
                              [  1,   0]])
        for i in range(len(self.test_coordinates)):
            coords = self.test_coordinates[i]
            calc_com = line_com(coords[0], coords[1], coords[2], coords[3], diameter=self.rod_diameter, density = self.rod_density)
            calc_com = calc_com / self.truth_mass[i]
            self.assertTrue(almost_equal(truth_com[i][0], calc_com[0], self.error_tol) and almost_equal(truth_com[i][1], calc_com[1], self.error_tol),
                            f"Rod center of mass case failed: {coords}. Expected {truth_com[i]} but output was {calc_com}")

        #     Message.info(f"Analysing rod with endpoints (y, z) at ({coords[0]}, {coords[1]}) and ({coords[2]}, {coords[3]})")
            
        #     if almost_equal(truth_com[i][0], calc_com[0], self.error_tol) and almost_equal(truth_com[i][1], calc_com[1], self.error_tol):
        #         Message.success("Passed Case")
        #     else:
        #         Message.error("Failed case")

        #     Message.debug(f"Truth Center of Mass:      ({truth_com[i][0]:9.4f}, {truth_com[i][1]:9.4f})")
        #     Message.debug(f"Calculated Center of Mass: ({calc_com[0]:9.4f}, {calc_com[1]:9.4f})")
        # print("")

    def test_rod_inertia(self):
        # Message.warning("Checking Inertia Tensor Calculations")
        # suffering
        truth_inertia = np.array([[1/3,   0, 1/3, 0, 0,   0],
                                  [1/3, 1/3,   0, 0, 0,   0],
                                  [1/3,   0, 1/3, 0, 0,   0],
                                  [1/3, 1/3,   0, 0, 0,   0],
                                  [2/3, 1/3, 1/3, 0, 0, 1/3],
                                  [2/3, 1/3, 1/3, 0, 0, 1/3],
                                  [5/3, 4/3, 1/3, 0, 0, 2/3],
                                  [4/3,   1, 1/3, 0, 0,   0],
                                  [4/3, 1/3,   1, 0, 0,   0]]).transpose() * self.truth_mass
        
        truth_inertia = truth_inertia.transpose()
        
        for i in range(len(self.test_coordinates)):
            coords = self.test_coordinates[i]
            calc_i = line_I(coords[0], coords[1], coords[2], coords[3], diameter=self.rod_diameter, density = self.rod_density)
            # Message.info(f"Analysing rod with endpoints (y, z) at ({coords[0]}, {coords[1]}) and ({coords[2]}, {coords[3]})")
            
            passed = True
            for j in range(6):
                if not almost_equal(truth_inertia[i][j], calc_i[j], self.error_tol):
                    passed = False

            self.assertTrue(passed, f"Failed rod inertia case: {coords}. Expected {truth_inertia[i]} but output was {calc_i}")    
        #     if passed:
        #         Message.success("Passed Case")
        #     else:
        #         Message.error("Failed case")
        #     Message.debug(f"Truth Inertia Tensor:      {truth_inertia[i]}")
        #     Message.debug(f"Calculated Inertia Tensor: {calc_i}")  
        # print("")  

if __name__ == "__main__":
    unittest.main()
