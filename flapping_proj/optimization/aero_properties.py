import numpy as np
from typing import Tuple
import numpy as np
from optimization.wing_classes import Wing
from optimization.geometry_classes import almost_equal
from utils.message import Message

def aero_properties(wing: Wing, n) -> Tuple[np.array, np.array]:
    start = .5/n
    del_y = (wing.leading_edge.y(1) - wing.leading_edge.y(0)) / n
    
    spar_cp = np.linspace(start, 1 - start, n) * del_y * n + wing.leading_edge.y(0)
    chord_cp = np.zeros_like(spar_cp)
    blade_area = np.zeros_like(spar_cp)
    
    # Since the leading and trailing edges are parametric, any given value of 
    # t may not map to the same value of y for the leading and trailing edge.
    # To get around this, the value of t to get a particular y value is found 
    # then the most extreme z value from the leading edge and trailing edge 
    # is taken to be where the end of the wing is
    for i in range(n):
        interest_y = spar_cp[i]
        t_upper = np.zeros(0, dtype=float)
        t_lower = np.zeros(0, dtype=float)
        tests = 100
        while (t_upper.size < 1) or (t_lower.size < 1):
            t = np.linspace(0, 1, tests)
            t_upper = t[np.where(almost_equal(interest_y, wing.leading_edge.y(t), 0.001))[0]]
            t_lower = t[np.where(almost_equal(interest_y, wing.trailing_edge.y(t), 0.001))[0]]
            tests *= 5
            if tests > 1e9:
                raise OverflowError(f"Unable to find valid parameters for blade calculations. Looking at {tests} inputs")
        
        z_upper = np.amax(wing.leading_edge.z(t_upper))
        z_lower = np.amin(wing.trailing_edge.z(t_lower))
        del_z = z_upper - z_lower
        
        chord_cp[i] = z_upper - 0.25 * del_z
        blade_area[i] = del_z * del_y
    
    return chord_cp, spar_cp, blade_area

if __name__ == "__main__":
    from optimization.wing_classes import TriWing

    curve = TriWing(0.0095, -0.003, 0.065, -0.070, 0.120, 0)

    print(aero_properties(curve, 20))

