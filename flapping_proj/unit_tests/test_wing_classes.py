import unittest
import numpy as np
from optimization.wing_classes import *
from utils.message import Message

def check_triangle_inequalities(Ixx, Iyy, Izz):
    return (Ixx + Iyy >= Izz) and (Iyy + Izz >= Ixx) and (Izz + Ixx >= Iyy)

class TestTriangleInequalities(unittest.TestCase):
    def test_valid_triangle_inequalities(self):
        self.assertTrue(check_triangle_inequalities(1, 1, 1))
        self.assertTrue(check_triangle_inequalities(2, 3, 4))
        self.assertTrue(check_triangle_inequalities(5, 5, 5))

    def test_invalid_triangle_inequalities(self):
        self.assertFalse(check_triangle_inequalities(1, 2, 4))
        self.assertFalse(check_triangle_inequalities(10, 1, 1))
        self.assertFalse(check_triangle_inequalities(0, 0, 1))

    def test_tri_wing(self):
        tri_wing = TriWing(0.001, -0.001, .05, -.025, .1, -.00025)
        #.i = [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        result = check_triangle_inequalities(tri_wing.I[0], tri_wing.I[1], tri_wing.I[2])
        Message.debug(f"I_origin: {tri_wing.I_origin[0]}, {tri_wing.I_origin[1]}, {tri_wing.I_origin[2]}, {tri_wing.I_origin[3]}, {tri_wing.I_origin[4]}, {tri_wing.I_origin[5]}")
        tensor_truth = np.array([148.692908, 6.875943, 141.835885, 0, 0, -3.224729]) / 1000**3 #1000**3 represents conversion from g mm**2 to kg m**2
        tensor_origin_truth = np.array([594.049086, 8.422495, 585.645511, 0, 0, 22.974025]) / 1000**3
        Message.debug(f"Tensor Truth Triangle Inequality Check: {check_triangle_inequalities(tensor_truth[0], tensor_truth[1], tensor_truth[2])}")

        com_truth = np.array([47.845714, -2.824405]) /1000
        Message.debug(f"Center of Mass truth: {com_truth}")
        Message.debug(f"Calculated COM: {tri_wing.com}")

        mass_truth = 0.193870 /1000
        Message.debug(f"Mass Truth: {mass_truth}")
        Message.debug(f"Calculated mass: {tri_wing.mass}")

        Message.debug(f"tensor_origin_truth: {tensor_origin_truth}")
        self.assertTrue(result, (f"Triangle Inequalities are not satisfied: {tri_wing.I[0]}, {tri_wing.I[1]}, {tri_wing.I[2]}"))
        self.assertTrue(tri_wing.mass > 0)
        self.assertTrue(tri_wing.I[0] > 0)
        self.assertTrue(tri_wing.I[1] > 0)
        self.assertTrue(tri_wing.I[2] > 0)
 
if __name__ == "__main__":
    unittest.main()
