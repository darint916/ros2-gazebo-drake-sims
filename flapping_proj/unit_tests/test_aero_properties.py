import unittest
import numpy as np
from optimization.component_classes import Curve
from utils.message import Message
from optimization.wing_classes import *
from optimization.aero_properties import *


class DumbRectange(Wing):
    def __init__(self, spar_length: float, chord_length: float):
        leading_edge = Curve(lambda t: spar_length * t**2,
                             lambda t: 2 * t * spar_length, lambda t: 0, lambda t: 0)
        trailing_edge = Curve(lambda t: spar_length * np.sin(t * np.pi / 2), lambda t: spar_length *
                              np.pi / 2 * np.cos(t * np.pi / 2), lambda t: -chord_length, lambda t: 0)
        super().__init__(leading_edge, trailing_edge)


class TestAeroProperties(unittest.TestCase):
    def test_rectangle_clac(self):
        cases = [RectWing(1, 1),
                 RectWing(2, 1),
                 RectWing(1, 2),
                 DumbRectange(1, 1)]
        n = 15
        start = .5/n
        truth_chord_cps = [np.ones(n) * -.25,
                           np.ones(n) * -.25,
                           np.ones(n) * -.5,
                           np.ones(n) * -.25]
        truth_spar_cps = [np.linspace(start, 1-start, n),
                          np.linspace(start, 1-start, n) * 2,
                          np.linspace(start, 1-start, n),
                          np.linspace(start, 1-start, n)]
        truth_blade_areas = [np.ones(n) / n,
                             np.ones(n) * 2 / n,
                             np.ones(n) * 2 / n,
                             np.ones(n) / n]

        for case in range(len(cases)):
            calc_chord, calc_spar, calc_area = aero_properties(cases[case], n)
            self.assertTrue(np.allclose(calc_spar, truth_spar_cps[case]),
                            f"Spar failed for rectangle {case}, expected {truth_spar_cps[case]}, got {calc_spar}")
            self.assertTrue(np.allclose(calc_chord, truth_chord_cps[case]),
                            f"Chord failed for rectange {case}, expected {truth_chord_cps[case]}, got {calc_chord}")
            self.assertTrue(np.allclose(calc_area, truth_blade_areas[case]),
                            f"Area failed for rectangle {case}, expected {truth_blade_areas[case]}, got {calc_area}")
            
    def test_internal_screaming(self):
        import optimization.approx_wing_amplitude as ah
        ah.flapping_amp()
