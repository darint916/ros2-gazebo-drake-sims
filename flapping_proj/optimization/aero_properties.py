import numpy as np

#bezier curve object and number of blades
from typing import Tuple
import numpy as np

def aero_properties(curve, n) -> Tuple[np.array, np.array]:
    start = .5/n
    t = np.linspace(start, 1-start, n)
    chord_cp = .25 * curve.z(t) #up/down, width, chord of wing
    spar_cp =  curve.y(t) #along leading edge of wing
    #x 0 into wing
    blade_area = abs(curve.z(t)) * (curve.y(t + start) - curve.y(t-start))
    return chord_cp, spar_cp, blade_area

if __name__ == "__main__":
    from inertial_properties import BezierCurve

    curve = BezierCurve(0.0095, -0.003, 0.065, -0.070, 0.120, -0.070, 0.130, -0.003)

    print(aero_properties(curve, 20))

