import numpy as np

#bezier curve object and number of blades
from typing import Tuple
import numpy as np
from optimization.wing_classes import Wing

def aero_properties(wing: Wing, n) -> Tuple[np.array, np.array]:
    start = .5/n
    t = np.linspace(start, 1-start, n)
    chord_cp = np.zeros_like(t)
    spar_cp = np.zeros_like(t)
    blade_area = np.zeros_like(t)
    for i in range(len(t)):
        chord_cp[i] = .25 * wing.trailing_edge.z(t[i]) #up/down, width, chord of wing
        spar_cp[i] =  wing.trailing_edge.y(t[i]) #along leading edge of wing
        #x 0 into wing
        blade_area[i] = abs(wing.trailing_edge.z(t[i])) * (wing.trailing_edge.y(t[i] + start) - wing.trailing_edge.y(t[i]-start))
    return chord_cp, spar_cp, blade_area

if __name__ == "__main__":
    from inertial_properties import BezierCurve

    curve = BezierCurve(0.0095, -0.003, 0.065, -0.070, 0.120, -0.070, 0.130, -0.003)

    print(aero_properties(curve, 20))

