import numpy as np
from numpy.core.multiarray import array as array
import scipy as sc
from optimization.geometry_classes import Geometry
from optimization.geometry_classes import *
from utils.message import Message
import sys


def check_triangle_inequalities(Ixx, Iyy, Izz):
    return (Ixx + Iyy >= Izz) and (Iyy + Izz >= Ixx) and (Izz + Ixx >= Iyy)

# If triangle test on the primary moments of inertia fails
# adds a very small percentage to the smallest moment of inertia.
# If the test is barely failing, this will make it pass.
# Increases the smallest moment by at least 1%


def inertia_modifier(inertia: np.array):
    max_increase = 100
    inertia = np.round(inertia, 12)
    # The error for a triangle inequality primary moments of inertia
    # These values are strictly positive for a valid tensor
    tri_error = np.array([inertia[0] + inertia[1] - inertia[2],
                          inertia[1] + inertia[2] - inertia[0],
                          inertia[0] + inertia[2] - inertia[1]])
    Message.info(f"Initial tensor values are {inertia}")
    min_inertia_index = np.argmin(inertia[0:2])
    i = 0
    while np.any(tri_error < sys.float_info.epsilon * 1e2):
        inertia[min_inertia_index] *= 1 + 1e-2
        if i > max_increase:
            Message.error(f"Inertia tensor could not be modified to satisfy triangle inequalities. Final tensor was {inertia}")
            break
        tri_error = np.array([inertia[0] + inertia[1] - inertia[2],
                              inertia[1] + inertia[2] - inertia[0],
                              inertia[0] + inertia[2] - inertia[1]])
        i+=1
    if i > 0:
        Message.info(
            f"Tensor modified from calculated value. Loop ran {i} times")
    return inertia


class Component():
    '''
    Basic component class
    Component.geo: a Geometry object that defines the shape of the object
    Component.mass: the mass of the Component in kg
    Component.com: the center of mass of the Component in the yz plane relative to the origin (m, m)
    Component.i: a 1x6 array representing [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] about the center of mass
    '''

    def __init__(self, geometry: Geometry, mass: float, com: np.array, inertia: np.array) -> None:
        self.geo = geometry
        self.mass = mass
        self.com = com
        self.I = inertia


class Film(Component):
    def __init__(self, upper: Curve, lower: Curve, thickness: float, density: float) -> None:
        bounded_region = BoundedRegion(upper, lower)
        super().__init__(bounded_region, bounded_region.area * thickness * density,
                         bounded_region.coa, thickness * density * bounded_region.second_moment)


class Sweep(Component):
    '''
    A profile that follows a curve. The profile should never self-intersect 
    and have a center of area at the origin.F 
    '''

    def __init__(self, path: Curve, profile: BoundedRegion, density: float) -> None:
        super().__init__(path, path.length * profile.area * density,
                         path.coa, path.second_moment * profile.area * density)


class Rod(Sweep):
    '''
    A slender cylindrical rod that follows a line in the yz plane
    '''

    def __init__(self, y0: float, z0: float, y1: float, z1: float, diameter: float, density: float) -> None:
        line = Line(y0, z0, y1, z1)
        profile = Circle(diameter)
        super().__init__(line, profile, density)
