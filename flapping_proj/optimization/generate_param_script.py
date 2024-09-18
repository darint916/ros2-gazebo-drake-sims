import optimization.wing_classes as wing_classes
import numpy as np
from optimization.aero_properties import aero_properties

test_wing = wing_classes.ThreeSegmentWing(
0.1,#     wing_length: float,
0.01,#     root_edge: float,
0,#     gap_size: float,
50 / 180 * np.pi,#     spar_angle_0: float,
0.04,#     spar_length_0: float,
40 / 180 * np.pi,#     spar_angle_1: float,
0.03#     spar_length_1: float
)
print("Inertia: ", test_wing.I)
print("Mass: ", test_wing.mass)
print("COM: ", test_wing.com)
chord_cp, spar_cp, blade_areas = aero_properties(
    test_wing, 20)
print("Chord CP: ", chord_cp)
print("Spar CP: ", spar_cp)
print("Blade Areaaight s: ", blade_areas)
