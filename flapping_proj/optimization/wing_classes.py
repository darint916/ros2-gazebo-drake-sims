from inertial_properties import*
import numpy as np
#wing with a trailing edge shaped like a bezier curve.
#there is the leading edge, a hinge bar, and trailing edge connected by a film 

class BezierWing():
    y0 = None
    z0 = None
    y1 = None
    z1 = None
    y2 = None
    z2 = None
    y3 = None
    z3 = None

    D_LE = 0.001 #m
    D_TE = 0.001 #m
    D_H  = 0.00025 #m

    #1d array of [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    I = None

    #Wing mass
    m = None
    
    def y(self, t):
        return (1 - t)**3 * self.y0 + 3*(1 - t)**2 * t * self.y1 + 3*(1 - t) * t**2 * self.y2 + t**3  * self.y3
    
    def z(self, t):
        return (1 - t)**3 * self.z0 + 3*(1 - t)**2 * t * self.z1 + 3*(1 - t) * t**2 * self.z2 + t**3  * self.z3
    
    def dy(self, t):
        return 3 * (1 - t)**2 * (self.y1 - self.y0) + 6 * (1 - t) * t * (self.y2 - self.y1) + 3 * t**2 * (self.y3 - self.y2)
    
    def dz(self, t):
        return 3 * (1 - t)**2 * (self.z1 - self.z0) + 6 * (1 - t) * t * (self.z2 - self.z1) + 3 * t**2 * (self.z3 - self.z2)
    
    def __init__(self, y_0, z_0, y_1, z_1, y_2, z_2, y_3, z_3):
        self.y0 = y_0
        self.z0 = z_0
        self.y1 = y_1
        self.z1 = z_1
        self.y2 = y_2
        self.z2 = z_2
        self.y3 = y_3
        self.z3 = z_3

        self.m = bez_wing_mass(self, d_le = self.D_LE, d_h = self.D_H, d_te = self.D_TE)

        self.I = bez_wing_I(self, self.m, d_le = self.D_LE, d_h = self.D_H, d_te = self.D_TE)
        

class TriWing():
    y0 = None
    z0 = None
    y1 = None
    z1 = None
    y2 = None
    z2 = None

    D_LE = 0.001 #m
    D_S = 0.0005 #m
    D_TE = 0.0005
    
    #1d array of [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    I = None
    #Wing mass
    m = None

    def y(self, t):
        t_adj = np.heaviside(t - .5, .5)
        return ((self.y1 - self.y0) * 2*t + self.y0) * (1 - t_adj) + t_adj * ((self.y2 - self.y1) * (2*t - 1) + self.y1)
    
    def z(self, t):
        t_adj = np.heaviside(t - .5, .5)
        return ((self.z1 - self.z0) * 2*t + self.z0) * (1 - t_adj) + t_adj * ((self.z2 - self.z1) * (2*t - 1) + self.z1)
        
    def dy(self, t):
        t_adj = np.heaviside(t - .5, .5)
        return (self.y1 - self.y0) * (1 - t_adj) + t_adj * (self.y2 - self.y1)
    
    def dz(self, t):
        t_adj = np.heaviside(t - .5, .5)
        return (self.z1 - self.z0) * (1 - t_adj) + t_adj * (self.z2 - self.z1)
        
    def __init__(self, y0, z0, y1, z1, y2, z2):
        self.y0 = y0
        self.z0 = z0
        self.y1 = y1
        self.z1 = z1
        self.y2 = y2
        self.z2 = z2
        
        self.m = line_m(y0, 0, y1, 0, diameter=self.D_LE) + line_m(y0, z0, y1, z1, diameter=self.D_TE) + line_m(y1, z0, y1, z1, diameter=self.D_S) + film_m(self)
        com = (line_com(y0, 0, y1, 0, diameter=self.D_LE) + line_com(y0, z0, y1, z1, diameter=self.D_TE) + line_com(y1, z0, y1, z1, diameter=self.D_S) + film_com(self)) / self.m
        com_r_sqr = com[0]**2 + com[1]**2

        I_origin = line_I(y0, 0, y1, 0, diameter=self.D_LE) + line_I(y0, z0, y1, z1, diameter=self.D_TE) + line_I(y1, z0, y1, z1, diameter=self.D_S) + film_I(self)
        self.I = I_origin - self.m * np.array([com_r_sqr, com_r_sqr - com[0]**2, com_r_sqr - com[1]**2, 0, 0, -com[0]*com[1]])
        
        
if __name__ == "__main__":
    import matplotlib.pyplot as plt


    tri_wing = TriWing(0, 0, .005, -.01, .01, -.005)
    # mass1 = film_m(tri_wing)
    # print(film_com(tri_wing) / mass1)

    # t = np.linspace(0, 1)
    # plt.plot(tri_wing.dy(t), tri_wing.dz(t), marker = "o")
    
    print(tri_wing.I)
    print(tri_wing.I[0] + tri_wing.I[1] - tri_wing.I[2])
    print(tri_wing.I[0] + tri_wing.I[2] - tri_wing.I[1])
    print(tri_wing.I[2] + tri_wing.I[1] - tri_wing.I[0])
