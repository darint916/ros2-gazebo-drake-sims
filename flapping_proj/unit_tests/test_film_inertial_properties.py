import unittest
import numpy as np
from optimization.inertial_properties import *
from utils.message import Message
# the fricken integration needs ot have a function bound for one of the 
# so int_0^1 int_0^y(t) f(y, z) dy dz/dt dt

class FilmCase():
    # master object to easily get all shape cases
    pass


class Rect1(FilmCase):
    name = "Rectangle (0, 0), (0, -1), (1, -1), (1, 0)"
    truth_mass = rho_mem * 1
    truth_com = [0.5, -0.5]
    truth_i = np.array([1/3*(1 + 1), 1/3, 1/3, 0, 0, -1/4]) * truth_mass

    def y(self, t):
        return t

    def z(self, t):
        return -1

    def dy(self, t):
        return 1

    def dz(self, t):
        return 0


class Rect2(FilmCase):
    name = "Rectangle (0, 0), (0, -1), (2, -1), (2, 0)"
    truth_mass = rho_mem * 2
    truth_com = [1, -0.5]
    truth_i = np.array([1/3*(4 + 2), 2/3, 4/3, 0, 0, -1]) * truth_mass

    def y(self, t):
        return 2*t

    def z(self, t):
        return -1

    def dy(self, t):
        return 2

    def dz(self, t):
        return 0


class Rect3(FilmCase):
    name = "Rectangle (1, 0), (1, -2), (2, -2), (2, 0)"
    truth_mass = rho_mem * 2
    truth_com = [1.5, -1]
    truth_i = np.array([1/3*(7*2 + 8), 8/3, 14/3, 0, 0, -3]) * truth_mass

    def y(self, t):
        return t+1

    def z(self, t):
        return -2

    def dy(self, t):
        return 1

    def dz(self, t):
        return 0

# calculaes a component of a moment of inertia for a right triangle


def right_triangle_moment(a0, a1, delta_b):
    delta_a = a1 - a0
    return delta_a*delta_b * (delta_a**2 / 3 + delta_a*a0 + a0**2)

# calculaes Iyz for a right triangle


def right_triangle_product(y0, y1, z0, z1):
    del_y = y1 - y0
    del_z = z1 - z0

    return del_y*del_z/3 + (del_y*z0 + del_z*y0)/2 + y0*z0


class Tri1(FilmCase):
    name = "triangle (0, 0), (0, -1), (1, 0)"
    truth_mass = rho_mem * .5
    truth_com = [1/3, -1/3]
    truth_i = np.array([right_triangle_moment(0, 1, 1) + right_triangle_moment(-1, 1, 1), right_triangle_moment(-1,
                       1, 1), right_triangle_moment(0, 1, 1), 0, 0, right_triangle_product(0, 1, -1, 0)]) * truth_mass

    def y(self, t):
        return t

    def z(self, t):
        return t-1

    def dy(self, t):
        return 1

    def dz(self, t):
        return 1


class Tri2(FilmCase):
    name = "triangle (1, 0), (1, -1), (2, 0)"
    truth_mass = rho_mem * .5
    truth_com = [4/3, -1/3]
    truth_i = np.array([right_triangle_moment(0, 1, 1) + right_triangle_moment(-1, 1, 1),
                        right_triangle_moment(-1, 1, 1), right_triangle_moment(0, 1, 1),
                        0, 0, right_triangle_product(0, 1, -1, 0)]) * truth_mass

    def y(self, t):
        return t

    def z(self, t):
        return t-1

    def dy(self, t):
        return 1

    def dz(self, t):
        return 1


class Tri3(FilmCase):
    name = "triangle (0, 0), (.5, -1), (1, 0)"
    truth_mass = rho_mem * .5
    truth_com = [0.5, -1/3]
    truth_i = np.array([right_triangle_moment(0, 1, 1) + right_triangle_moment(-1, 1, 1),
                        right_triangle_moment(-1, 1, 1),
                        right_triangle_moment(0, 1, 1),
                        0, 0, right_triangle_product(0, 1, -1, 0)]) * truth_mass

    def y(self, t):
        return t

    def z(self, t):
        return -2*t if t <= .5 else 2*t-2

    def dy(self, t):
        return 1

    def dz(self, t):
        return -2 if t <= .5 else 2


class TestFilmInertiaProperties(unittest.TestCase):

    def test_film_mass(self):
        Message.error(FilmCase.__subclasses__()[0].name)


if __name__ == "__main__":
    Message.debug(FilmCase.__subclasses__)
