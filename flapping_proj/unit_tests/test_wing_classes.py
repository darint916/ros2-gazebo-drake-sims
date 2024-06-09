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
        tri_wing = TriWing(0, 0, .005, -.01, .01, -.005)
        #.i = [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        result = check_triangle_inequalities(tri_wing.I[0], tri_wing.I[1], tri_wing.I[2])
        self.assertTrue(result, (f"Triangle Inequalities are not satisfied: {tri_wing.I[0]}, {tri_wing.I[1]}, {tri_wing.I[2]}"))
        self.assertTrue(tri_wing.m > 0)
        self.assertTrue(tri_wing.I[0] > 0)
        self.assertTrue(tri_wing.I[1] > 0)
        self.assertTrue(tri_wing.I[2] > 0)
        
if __name__ == "__main__":
    unittest.main()