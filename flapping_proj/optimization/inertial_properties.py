import numpy as np
import scipy as sc

#carbon fiber rod diameters
D_LE = 0.001 #m
D_TE = 0.001 #m
D_H  = 0.00025 #m

#linear densities of carbon fiber rods
rho_le = np.pi / 4 * D_LE**2 * 1854.55 #kg/m
rho_te = np.pi / 4 * D_TE**2 * 1854.55 #kg/m
rho_h = np.pi / 4 * D_H * 1854.55 #kg/m

#area density for 12 micron PET sheet
rho_mem = 0.05 * 0.0005 / 0.00254**2 / 2.2 #kg/m**2

class BezierCurve():
    y0 = None
    z0 = None
    y1 = None
    z1 = None
    y2 = None
    z2 = None
    y3 = None
    z3 = None
    
    def B_y(self, t):
        return (1 - t)**3 * self.y0 + 3*(1 - t)**2 * t * self.y1 + 3*(1 - t) * t**2 * self.y2 + t**3  * self.y3
    
    def B_z(self, t):
        return (1 - t)**3 * self.z0 + 3*(1 - t)**2 * t * self.z1 + 3*(1 - t) * t**2 * self.z2 + t**3  * self.z3
    
    def dB_y(self, t):
        return 3 * (1 - t)**2 * (self.y1 - self.y0) + 6 * (1 - t) * t * (self.y2 - self.y1) + 3 * t**2 * (self.y3 - self.y2)
    
    def dB_z(self, t):
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

def wing_I(curve):
    #leading edge
    I_LE = np.array([1/3 * rho_le * curve.y3 * (3 * (D_LE / 2)**2 + curve.y3**2), .5 * rho_le * (D_LE / 2)**3, 1/3 * rho_le * curve.y3 * (3 * (D_LE / 2)**2 + curve.y3**2), 0, 0, 0])
    #hinge
    I_H = np.array([rho_h * (1/3 * curve.y3**3 + curve.z0**2 * curve.y3 - (1/3 * (curve.y0 + D_TE + D_H)**3 + curve.z0**2 * (curve.y0  + D_TE + D_H))), 
                    rho_h * curve.z0 * (curve.y3 - (curve.y0 + D_TE + D_H)), 
                    1/3 * rho_h * (curve.y3**3 - (curve.y0 + D_TE + D_H)**3),
                    0, 
                    0, 
                    0.5 * rho_h*curve.z0*(curve.y3 - (curve.y0  + D_TE + D_H))])
    
    t1 = 2 * curve.z1 - curve.z0 - curve.z2 

    t2 = curve.z1**2 - curve.z1 * curve.z2 - curve.z1 * curve.z3 + curve.z0 * curve.z3 + curve.z2**2 - curve.z0 * curve.z2 

    t3 = 3 * curve.z1 - curve.z0 - 3 * curve.z2 + curve.z3 

    t_max = 0.5 if abs(t3) < 1e-6 else t1 + np.sqrt(t2) / t3

    #trailing edge
    I_TE = np.array([sc.integrate.quad(lambda t: (curve.B_y(t)**2 + curve.B_z(t)**2) * dm_te(curve, t), 0, t_max)[0],
                     sc.integrate.quad(lambda t: (curve.B_z(t)**2) * dm_te(curve, t), 0, t_max)[0],
                     sc.integrate.quad(lambda t: (curve.B_y(t)**2) * dm_te(curve, t), 0, t_max)[0],
                     0,
                     0,
                     sc.integrate.quad(lambda t: (curve.B_y(t) * curve.B_z(t)) * dm_te(curve, t), 0, t_max)[0]])
    
    #membrane
    I_film = np.array([rho_mem * sc.integrate.quad(lambda t: (curve.B_z(t)*curve.B_y(t)**2 + 1/3 * curve.B_z(t)**3) * curve.dB_y(t), 0, 1)[0],
                       1/3 * rho_mem * sc.integrate.quad(lambda t: curve.B_z(t)**3 * curve.dB_y(t), 0, 1)[0],
                       rho_mem * sc.integrate.quad(lambda t: curve.B_y(t)**2 * curve.B_z(t) * curve.dB_y(t), 0, 1)[0],
                       0,
                       0,
                       0.5 * rho_mem * sc.integrate.quad(lambda t: curve.B_y(t) * curve.B_z(t)**2 * curve.dB_y(t), 0, 1)[0]])
    
    return I_LE + I_H + I_TE + I_film
    
def dm_te(curve, t):
        return rho_te * np.sqrt(curve.dB_z(t)**2 + curve.dB_y(t)**2)

def wing_mass(curve):
    m_LE = curve.y3 * rho_le
    m_h = (curve.y3 - (curve.y0 + D_H + D_TE)) * rho_h
    m_film = sc.integrate.quad(lambda t: curve.B_z(t) * curve.dB_y(t), 0, 1)

    t1 = 2 * curve.z1 - curve.z0 - curve.z2 
    t2 = curve.z1**2 - curve.z1 * curve.z2 - curve.z1 * curve.z3 + curve.z0 * curve.z3 + curve.z2**2 - curve.z0 * curve.z2 
    t3 = 3 * curve.z1 - curve.z0 - 3 * curve.z2 + curve.z3 

    t_max = 0.5 if abs(t3) < 1e-6 else t1 + np.sqrt(t2) / t3

    m_te = sc.integrate.quad(lambda t: dm_te(curve, t), 0, t_max)

    return m_LE + m_h + m_film + m_te