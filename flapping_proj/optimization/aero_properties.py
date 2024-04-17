import numpy as np

#bezier curve object and number of blades
def aero_properties(curve, n):
    start = .5/n
    t = np.linspace(start, 1-start, n)
    center_pressure = .25 * curve.B_z(t)
    blade_area = abs(curve.B_z(t)) * (curve.B_y(t + start) - curve.B_y(t-start))
    return center_pressure, blade_area

if __name__ == "__main__":
    from inertial_properties import BezierCurve

    curve = BezierCurve(0.0095, -0.003, 0.065, -0.070, 0.120, -0.070, 0.130, -0.003)

    print(aero_properties(curve, 20))
