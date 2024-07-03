import unittest
import numpy as np
from optimization.inertial_properties import *
from utils.message import Message
# the fricken integration needs ot have a function bound for one of the 
# so int_0^1 int_0^y(t) f(y, z) dy dz/dt dt

def almost_equal(a, b, error_percent):
    return abs(a - b) <= abs(a*error_percent)

class FilmCase():
    # master object to easily get all shape cases
    # this did not work for some reason
    pass


class Rect1(FilmCase):
    name = "Rectangle (0, 0), (0, -1), (1, -1), (1, 0)"
    truth_mass = rho_mem * 1
    truth_com = [0.5, -0.5]
    truth_i = np.array([2/3, 1/3, 1/3, 0, 0, -1/4]) * truth_mass

    def y(self, t):
        return t

    def lower_z(self, t):
        return -1

    def dy(self, t):
        return 1

    def lower_dz(self, t):
        return 0
    
    def upper_z(self, t):
        return 0
    
    def upper_dz(self, t):
        return 0


class Rect2(FilmCase):
    name = "Rectangle (0, 0), (0, -1), (2, -1), (2, 0)"
    truth_mass = rho_mem * 2
    truth_com = [1, -0.5]
    truth_i = np.array([10/3, 2/3, 8/3, 0, 0, -1]) * truth_mass

    def y(self, t):
        return 2*t

    def lower_z(self, t):
        return -1

    def dy(self, t):
        return 2

    def lower_dz(self, t):
        return 0
    
    def upper_z(self, t):
        return 0
    
    def upper_dz(self, t):
        return 0


class Rect3(FilmCase):
    name = "Rectangle (1, 0), (1, -2), (2, -2), (2, 0)"
    truth_mass = rho_mem * 2
    truth_com = [1.5, -1]
    truth_i = np.array([1/3*(7*2 + 8), 8/3, 14/3, 0, 0, -3]) * truth_mass

    def y(self, t):
        return t+1

    def lower_z(self, t):
        return -2

    def dy(self, t):
        return 1

    def lower_dz(self, t):
        return 0
    
    def upper_z(self, t):
        return 0
    
    def upper_dz(self, t):
        return 0

class Tri1(FilmCase):
    name = "Triangle (0, 0), (0, -1), (1, 0)"
    truth_mass = rho_mem * .5
    truth_com = [1/3, -1/3]
    truth_i = np.array([2/12,
                        1/12,
                        1/12,
                        0,
                        0,
                        -1/24]) * truth_mass

    def y(self, t):
        return t

    def lower_z(self, t):
        return t-1

    def dy(self, t):
        return 1

    def lower_dz(self, t):
        return 1
    
    def upper_z(self, t):
        return 0
    
    def upper_dz(self, t):
        return 0


class Tri2(FilmCase):
    name = "Triangle (1, 0), (1, -1), (2, 0)"
    truth_mass = rho_mem * .5
    truth_com = [4/3, -1/3]
    truth_i = np.array([1,
                        1/12,
                        11/12,
                        0,
                        0,
                        -5/24]) * truth_mass

    def y(self, t):
        return t+1

    def lower_z(self, t):
        return t-1

    def dy(self, t):
        return 1

    def lower_dz(self, t):
        return 1
    
    def upper_z(self, t):
        return 0
    
    def upper_dz(self, t):
        return 0


class Tri3(FilmCase):
    name = "Triangle (0, 0), (.5, -1), (1, 0)"
    truth_mass = rho_mem * .5
    truth_com = [0.5, -1/3]
    truth_i = np.array([7/96+5/32,
                        1/12,
                        1/32+11/96,
                        0, 
                        0, 
                        -5/96-1/32]) * truth_mass

    def y(self, t):
        return t

    def lower_z(self, t):
        return -2*t if t <= .5 else 2*t-2

    def dy(self, t):
        return 1

    def lower_dz(self, t):
        return -2 if t <= .5 else 2
    
    def upper_z(self, t):
        return 0
    
    def upper_dz(self, t):
        return 0
    
class Para1(FilmCase):
    name = "Parabola through (0, 0), (1, -1), (2, 0)"
    truth_mass = rho_mem * (4/3)
    truth_com = [1, -.4]
    truth_i = np.array([32/105+8/5,
                        32/105,
                        8/5,
                        0,
                        0,
                        -8/15]) * truth_mass
    
    def y(self, t):
        return 2*t
    
    def lower_z(self, t):
        return 4*t**2 - 4*t
    
    def dy(self, t):
        return 2
    
    def lower_dz(self, t):
        8*t - 4
        
    def upper_z(self, t):
        return 0
    
    def upper_dz(self, t):
        return 0


class TestFilmInertiaProperties(unittest.TestCase):
    cases = [Rect1(), Rect2(), Rect3(), Tri1(), Tri2(), Tri3(), Para1()]
    error_tol = 0.001 #.1% as a decimal
    
    def test_film_mass(self):
        for i in self.cases:
            calc_mass = film_m(i)
            self.assertAlmostEqual(i.truth_mass, calc_mass, delta=self.error_tol*i.truth_mass, msg=f"Incorrect Mass in f{i.name}: expected {i.truth_mass} calculated {calc_mass}")
            
    def test_film_com(self):
        for i in self.cases:
            calc_com = film_com(i) / i.truth_mass
            self.assertTrue(almost_equal(i.truth_com[0], calc_com[0], self.error_tol) and almost_equal(i.truth_com[1], calc_com[1], self.error_tol), 
                            msg=f"Film center of mass case failed: {i.name}. Expected {i.truth_com} but output was {calc_com}")
            
    def test_film_inertia(self):
        for i in self.cases:
            calc_i = film_I(i)
            
            passed = True
            for j in range(6):
                if not almost_equal(i.truth_i[j], calc_i[j], self.error_tol):
                    passed = False

            self.assertTrue(passed, f"Failed rod inertia case: {i.name}. Expected {i.truth_i} but output was {calc_i}") 

if __name__ == "__main__":
    Message.debug(FilmCase.__subclasses__)
