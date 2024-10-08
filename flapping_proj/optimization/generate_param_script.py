import optimization.wing_classes as wing_classes
import numpy as np
from optimization.aero_properties import aero_properties
total_rotate = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
print("Total Rotate:\n", total_rotate)
# x_rot = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
# y_rot = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
# z_rot = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
# print(z_rot @ y_rot)
# print("rot test d")
test_wing = wing_classes.ThreeSegmentWing(
0.1,#     wing_length: float,
0.01,#     root_edge: float,
0,#     gap_size: float,
50 / 180 * np.pi,#     spar_angle_0: float,
0.04,#     spar_length_0: float,
40 / 180 * np.pi,#     spar_angle_1: float,
0.03#     spar_length_1: float
)
print("Inertia (leading edge on y axis, wing on -yz): ixx iyy izz ixy ixz iyz", test_wing.I)
print("Inerita change to wing on xy plane: ixx iyy izz ixy ixz iyz\n", total_rotate @ np.array([[test_wing.I[0], test_wing.I[3], test_wing.I[4]], [test_wing.I[3], test_wing.I[1], test_wing.I[5]], [test_wing.I[4], test_wing.I[5], test_wing.I[2]]]) @ total_rotate.T)
print("Mass: ", test_wing.mass)
print("COM: (mult y by 10)", test_wing.com)
chord_cp, spar_cp, blade_areas = aero_properties(
    test_wing, 20)
print("Chord CP: ", chord_cp)
print("Spar CP: ", spar_cp)
print("Blade Areas: ", blade_areas)
total_cp = [item for (x,y) in zip(spar_cp, chord_cp ) for item in (x, y * 10, 0)]
print("CP: ", total_cp)
print("up vec: ", [0, 0, 1] * len(spar_cp))
print("blades: ", len(spar_cp) )