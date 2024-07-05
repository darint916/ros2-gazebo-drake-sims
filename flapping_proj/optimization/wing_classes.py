from flapping_proj.optimization.component_classes import Curve
from flapping_proj.optimization.geometry_classes import Curve
from optimization.inertial_properties import *
import numpy as np
import matplotlib.pyplot as plt
from optimization.geometry_classes import *
from optimization.component_classes import *

RHO_CF = 1854.55  # kg/m**3
RHO_PET = 1383.995  # kg/m**3


# generic wing object
# assumes all components are placed correctly in the yz plane
# This is a structure with a single film supported by
# a collection of rigid members
class Wing():
    # leading_edge is a curve (must be a fully solid member in reality)
    # trailing edge is a curve (may not be a fully solid member)
    # leading_edge(y) >= trailing_edge(y).
    # sweeps is an array of Sweep instances
    # sweeps must include the sweep forming the leading edge
    def __init__(self, leading_edge: Curve, trailing_edge: Curve, sweeps=[]):
        self.components = [Film(leading_edge, trailing_edge, 12e-6, RHO_PET)]
        for i in sweeps:
            self.components.append(i)

        self.mass = 0
        for i in self.components:
            self.mass += i.mass

        self.com = np.zeros(2)
        for i in self.components:
            self.com += i.com * i.mass
        self.com /= self.mass

        self.I = np.zeros(6)
        for i in self.components:
            del_y = i.com[0] - self.com[0]
            del_z = i.com[1] - self.com[1]
            displacement_radius = (del_y)**2 + (del_z)**2
            self.I += i.I + i.mass * \
                np.array([displacement_radius, displacement_radius - del_y **
                         2, displacement_radius - del_z, 0, 0, -del_y * del_z])

        self.I = inertia_modifier(self.I)

        com_magnitude_sqr = self.com[0]**2 + self.com[1]**2
        self.I_origin = self.I + self.mass * \
            np.array([com_magnitude_sqr, com_magnitude_sqr - self.com[0]**2,
                     com_magnitude_sqr - self.com[1], 0, 0, -self.com[0]*self.com[1]])

# depriciated
class BezierWing():
    def __init__(self, y_0, z_0, y_1, z_1, y_2, z_2, y_3, z_3):
        print("WARNING: BezierWing uses depriciated components and is not yet updated to Component internal format.")
        self.y0 = y_0
        self.z0 = z_0
        self.y1 = y_1
        self.z1 = z_1
        self.y2 = y_2
        self.z2 = z_2
        self.y3 = y_3
        self.z3 = z_3

        self.m = bez_wing_mass(self, d_le=self.D_LE,
                               d_h=self.D_H, d_te=self.D_TE)

        self.I = bez_wing_I(self, self.m, d_le=self.D_LE,
                            d_h=self.D_H, d_te=self.D_TE)

    def y(self, t):
        return (1 - t)**3 * self.y0 + 3*(1 - t)**2 * t * self.y1 + 3*(1 - t) * t**2 * self.y2 + t**3 * self.y3

    def dy(self, t):
        return 3 * (1 - t)**2 * (self.y1 - self.y0) + 6 * (1 - t) * t * (self.y2 - self.y1) + 3 * t**2 * (self.y3 - self.y2)

    def lower_z(self, t):
        return (1 - t)**3 * self.z0 + 3*(1 - t)**2 * t * self.z1 + 3*(1 - t) * t**2 * self.z2 + t**3 * self.z3

    def lower_dz(self, t):
        return 3 * (1 - t)**2 * (self.z1 - self.z0) + 6 * (1 - t) * t * (self.z2 - self.z1) + 3 * t**2 * (self.z3 - self.z2)

    def upper_z(self, t):
        return 0

    def upper_dz(self, t):
        return 0

# Inertial Return: 1d array of [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]


class TriWing(Wing):
    def __init__(self, y0, z0, y1, z1, y2, z2):
        self.y0 = np.round(y0, 12)
        self.z0 = np.round(z0, 12)
        self.y1 = np.round(y1, 12)
        self.z1 = np.round(z1, 12)
        self.y2 = np.round(y2, 12)
        self.z2 = np.round(z2, 12)

        # m leading edge carbon fiber rod diameter
        self.leading_edge_rod_diameter = 0.001
        self.spar_rod_diameter = 0.0005  # m Spar carbon fiber rod diameter
        # m trailing edge carbon fiber rod diameter
        self.trailing_edge_rod_diameter = 0.0005

        leading_edge = Line(self.y0, 0, self.y2, 0)
        trailing_edge = LineSegments([self.y0, self.y1, self.y2], [
                                     self.z0, self.z1, self.z2])
        rods = [Rod(0.002, 0, self.y2, 0, self.leading_edge_rod_diameter, RHO_CF),
                Rod(self.y0, self.z0, self.y1, self.z1,
                    self.trailing_edge_rod_diameter, RHO_CF),
                Rod(self.y1, self.z0, self.y1, self.z1, self.spar_rod_diameter, RHO_CF)]
        super().__init__(leading_edge, trailing_edge, rods)


class StraightButterflyWing(Wing):
    def __init__(self, film_start: float, leading_edge_length: float, leading_edge_angle: float, mid_bar_length: float, mid_bar_angle: float, trailing_edge_length: float, trailing_edge_angle: float):
        raise Exception("Not yet supported")
        super().__init__(leading_edge, trailing_edge, sweeps)


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    tri_wing = TriWing(0.03, 0, .1485, -.2, .15, 0)
    # mass1 = film_m(tri_wing)
    # print(film_com(tri_wing) / mass1)

    t = np.linspace(0, 1)
    # plt.plot(tri_wing.y(t), tri_wing.z(t))
    # plt.plot(tri_wing.y(t), tri_wing.dz(t) / tri_wing.dy(t), marker = "o")

    print(tri_wing.mass)
    print(tri_wing.com)
    print(tri_wing.I)
    print(tri_wing.I_origin)
    print(tri_wing.I[0] + tri_wing.I[1] - tri_wing.I[2])
    print(tri_wing.I[0] + tri_wing.I[2] - tri_wing.I[1])
    print(tri_wing.I[2] + tri_wing.I[1] - tri_wing.I[0])
