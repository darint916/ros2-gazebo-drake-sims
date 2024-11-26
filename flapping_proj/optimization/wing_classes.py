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
    # inertia given as  ixx iyy izz ixy ixz iyz
    def __init__(self, leading_edge: Curve, trailing_edge: Curve, sweeps=[]):
        self.components = [Film(leading_edge, trailing_edge, 12e-6, RHO_PET)]
        self.leading_edge = leading_edge
        self.trailing_edge = trailing_edge
        for i in sweeps:
            self.components.append(i)

        self.mass = 0
        self.com = np.zeros(2)
        self.I = np.zeros(6)
        for i in self.components:
            self.mass += i.mass
            self.com += i.com * i.mass
        self.com /= self.mass
        
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
            
    def display_wing(self):
        t = np.linspace(0, 1, 1000)
        fig, ax = plt.subplots()
        films = []
        sweeps = []
        for i in self.components:
            if type(i) == Film:
                films.append(i)
            elif type(i) == Sweep:
                sweeps.append(i)
            else:
                raise Exception("Unepected component type encountered while plotting the wing")
        
        for i in films:
            ax.plot(i.geo.upper.y(t), i.geo.upper.z(t), alpha = .5, color="blue")
            ax.plot(i.geo.lower.y(t), i.geo.lower.z(t), alpha = .5, color="blue")
        for i in sweeps:
            ax.plot(i.geo.y(t), i.geo.z(t), color = "black")

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
        trailing_edge = LineSegments(np.array([self.y0, self.y1, self.y2]), np.array([
                                     self.z0, self.z1, self.z2]))
        rods = [Rod(0.002, 0, self.y2, 0, self.leading_edge_rod_diameter, RHO_CF),
                Rod(self.y0, self.z0, self.y1, self.z1,
                    self.trailing_edge_rod_diameter, RHO_CF),
                Rod(self.y1, self.z0, self.y1, self.z1, self.spar_rod_diameter, RHO_CF)]
        super().__init__(leading_edge, trailing_edge, rods)


class RightWing(Wing):
    '''
    A wing in the shape of a right triangle
    '''

    def __init__(self, spar_length: float, chord_length: float):
        leading_edge = Line(0, 0, spar_length, 0)
        trailing_edge = Line(0, -chord_length, spar_length, 0)
        sweeps = [Sweep(leading_edge, Circle(0.001), RHO_CF)]
        super().__init__(leading_edge, trailing_edge, sweeps)


class RectWing(Wing):
    '''
    A wing in the shape of a rectangle
    Mostly a debug class as making this wing as is 
    would not function well
    '''

    def __init__(self, spar_length: float, chord_length: float):
        leading_edge = Line(0, 0, spar_length, 0)
        trailing_edge = Line(0, -chord_length, spar_length, -chord_length)
        sweeps = [Sweep(leading_edge, Circle(0.001), RHO_CF)]
        super().__init__(leading_edge, trailing_edge, sweeps)


class StraightButterflyWing(Wing):
    def __init__(self, film_start: float, leading_edge_length: float, leading_edge_angle: float, mid_bar_length: float, mid_bar_angle: float, trailing_edge_length: float, trailing_edge_angle: float):
        raise Exception("Not yet supported")
        super().__init__(leading_edge, trailing_edge, sweeps)


class ThreeSegmentWing(Wing):
    '''
    A wing that has a straight leading edge and two spars evenly 
    spaced along the leading edge that support the structure of the wing. 
    '''

    def __init__(self, wing_length: float, root_edge: float, gap_size: float, spar_angle_0: float, spar_length_0: float, spar_angle_1: float, spar_length_1: float):
        leading_edge_diameter = 0.001  # m
        spar_diameter = 0.0005  # m
        leading_edge_bar = Rod(0.002, 0, wing_length, 0,
                               leading_edge_diameter, RHO_CF)

        spar_0_pts = [wing_length / 3,
                      -gap_size,
                      wing_length / 3 + spar_length_0 * np.cos(spar_angle_0),
                      -gap_size - spar_length_0 * np.sin(spar_angle_0)]
        spar_0 = Rod(spar_0_pts[0], spar_0_pts[1],
                     spar_0_pts[2], spar_0_pts[3], spar_diameter, RHO_CF)

        spar_1_pts = [2*wing_length / 3,
                      -gap_size,
                      2*wing_length / 3 + spar_length_1 * np.cos(spar_angle_1),
                      -gap_size - spar_length_1 * np.sin(spar_angle_1)]
        spar_1 = Rod(spar_1_pts[0], spar_1_pts[1],
                     spar_1_pts[2], spar_1_pts[3], spar_diameter, RHO_CF)
        

        if spar_0_pts[2] > spar_1_pts[2]:
            raise ValueError(
                "Spar 0 extends beyond spar 1, which is not allowed.")
        elif spar_1_pts[2] > wing_length:
            leading_edge = LineSegments(np.array(
                [root_edge, wing_length, spar_1_pts[2]]), np.array([0, 0, spar_1_pts[3]]))
            trailing_edge = LineSegments(np.array(
                [root_edge, spar_0_pts[2], spar_1_pts[2]]), np.array([0, spar_0_pts[3], spar_1_pts[3]]))
        else:
            leading_edge = Line(root_edge, 0, wing_length, 0)
            trailing_edge = LineSegments(np.array([root_edge, spar_0_pts[2], spar_1_pts[2], wing_length]), np.array(
                [0, spar_0_pts[3], spar_1_pts[3], 0]))

        super().__init__(leading_edge, trailing_edge,
                         [leading_edge_bar, spar_0, spar_1])


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
