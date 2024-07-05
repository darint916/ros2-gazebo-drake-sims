import numpy as np
from numpy.core.multiarray import array as array
import scipy as sc
from flapping_proj.optimization.geometry_classes import Geometry
from optimization.geometry_classes import *

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
        self.i = inertia
        
class Film(Component):
    def __init__(self, upper: Curve, lower: Curve, thickness: float, density: float) -> None:
        bounded_region = Bounded_Region(upper, lower)
        super().__init__(bounded_region, bounded_region.area * thickness * density, bounded_region.coa, thickness * density * bounded_region.second_moment)
    
class Sweep(Component):
    def __init__(self, path: Curve, profile: Bounded_Region, density: float) -> None:
        super().__init__(path, path.length * profile.area * density, path.coa, path.second_moment * profile.area * density)
        
class Rod(Sweep):
    def __init__(self, y0: float, z0: float, y1: float, z1: float, diameter: float, density: float) -> None:
        line = Line(y0, z0, y1, z1)
        profile = Circle(diameter)
        super().__init__(line, line.length * profile.area * density, line.coa, line.second_moment * profile.area * density)
