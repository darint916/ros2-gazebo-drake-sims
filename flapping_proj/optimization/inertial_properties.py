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
rho_mem = 0.05 * 0.0005 / 0.0254**2 / 2.2 #kg/m**2

#calculates moments of inertia for a slender rod in the yz plane
def line_I(y0, z0, y1, z1, com = np.array([0, 0]), diameter = D_LE, density = 1854.55) -> np.array:
    rho = np.pi / 4 * diameter**2 * density #kg/m
    del_y = y1 - y0
    del_z = z1 - z0

    line_length = np.sqrt(del_y**2 + del_z**2)
    
    transformed_y0 = y0 - com[0]
    transformed_z0 = z0 - com[1]

    y_integrated = del_y**2 / 3 + del_y * transformed_y0 + transformed_y0**2
    z_integrated = del_z**2 / 3 + del_z * transformed_z0 + transformed_z0**2

    I = np.array([y_integrated + z_integrated, 
                  z_integrated, 
                  y_integrated, 
                  0, 
                  0, 
                  (del_y*del_z/3 + (del_z*transformed_y0 + del_y*transformed_z0)/2 + transformed_y0*transformed_z0)]) * rho * line_length
    return I

#calculates the mass of a rod in the yz plane

def line_m(y0, z0, y1, z1, diameter = D_LE, density = 1854.55) -> float: #value tested
    rho = np.pi / 4 * diameter**2 * density #kg/m

    del_y = y1 - y0
    del_z = z1 - z0

    line_length = np.sqrt(del_y**2 + del_z**2)
    return rho * line_length

# returns the center of mass of a cylindral rod multiplied by 
# the rod mass so it can be divided by the overall system mass
# to get a system center of mass
# [y, z]
def line_com(y0, z0, y1, z1, diameter = D_LE, density = 1854.55) -> np.array:
    rho = np.pi / 4 * diameter**2 * density #kg/m
    del_y = y1 - y0
    del_z = z1 - z0

    line_length = np.sqrt(del_y**2 + del_z**2)
    return np.array([y0 + del_y/2, z0+del_z/2]) * line_length*rho

#calculates the moments of inertia for a slender rod in the yz plane that follows a curve
def curve_I(curve, com = np.array([0, 0]), diameter = D_TE, t_max = 1, density = 1854.55) -> np.array:
    rho = np.pi / 4 * diameter**2 * density #kg/m
    I = np.array([sc.integrate.quad(lambda t: ((curve.y(t) - com[0])**2 + (curve.lower_z(t) - com[1])**2) * dm_curve(curve, t, rho), 0, t_max)[0],
                  sc.integrate.quad(lambda t: (curve.lower_z(t) - com[1])**2 * dm_curve(curve, t, rho), 0, t_max)[0],
                  sc.integrate.quad(lambda t: (curve.y(t) - com[0])**2 * dm_curve(curve, t, rho), 0, t_max)[0],
                  0,
                  0,
                  sc.integrate.quad(lambda t: ((curve.y(t) - com[0]) * (curve.lower_z(t) - com[1])) *dm_curve(t), 0, t_max)[0]])
    return I

#calculates the mass of a rod following a curve in the yz plane
def curve_m(curve, diameter = D_TE, t_max = 1, density = 1854.55) -> float:
    rho = np.pi / 4 * diameter**2 * density #kg/m
    return sc.integrate.quad(lambda t: dm_curve(curve, t, rho), 0, t_max)[0]

# returns the center of mass of a cylindral rod following 
# a curve multiplied by the curve mass so it can be divided 
# by the overall system mass to get a system center of mass    
# [y, z]

def curve_com(curve, diameter = D_TE, t_max = 1, density = 1854.55) -> np.array:
    rho = np.pi / 4 * diameter**2 * density #kg/m
    return np.array([sc.integrate.quad(lambda t: curve.y(t) * dm_curve(curve, t, rho), 0, t_max)[0],
                     sc.integrate.quad(lambda t: curve.lower_z(t) * dm_curve(curve, t, rho), 0, t_max)[0]])

#calculates the moment of inertia for a planar film bounded by a curve in the yz plane.
#Assumes the curve is below the y axis
def film_I(curve, com = np.array([0, 0]), rho = rho_mem) -> np.array:
    mass = film_m(curve, rho=rho)
    y_sqr_int = sc.integrate.quad(lambda t: (curve.y(t) - com[0])**2 * (curve.upper_z(t) - curve.lower_z(t)) * curve.dy(t), 0, 1)[0]
    z_sqr_int = 1/3 * sc.integrate.quad(lambda t: ((curve.upper_z(t) - com[1])**3 - (curve.lower_z(t) - com[1])**3) * curve.dy(t), 0, 1)[0]
    product_int = 0.5 * sc.integrate.quad(lambda t: (curve.y(t) - com[0]) * ((curve.upper_z(t)**2 - com[1])**2 - (curve.lower_z(t) - com[1])**2) * curve.dy(t), 0, 1)[0]
    I = np.array([y_sqr_int +  z_sqr_int,
                  z_sqr_int,
                  y_sqr_int,
                  0,
                  0,
                  product_int])
    return I * mass

#calculates the mass of a planar film bounded by a curve in the yz plane.
#Assumes the curve is below the y axis
def film_m(curve, rho = rho_mem) -> float:
    return rho * sc.integrate.quad(lambda t: np.abs((curve.upper_z(t) - curve.lower_z(t)) * curve.dy(t)), 0, 1)[0]

# returns the center of mass of a thin film following 
# a curve multiplied by the curve mass so it can be divided 
# by the overall system mass to get a system center of mass 
# [y, z]
# technically this calculates the moment with respect to an axis
def film_com(curve, rho = rho_mem) -> np.array: #value tested
    return np.array([rho * sc.integrate.quad(lambda t: (curve.upper_z(t) - curve.lower_z(t)) * curve.y(t) * curve.dy(t), 0, 1)[0],
                     rho / 2 * sc.integrate.quad(lambda t: (curve.upper_z(t)**2 - curve.lower_z(t)**2) * curve.dy(t), 0, 1)[0]])

def bez_wing_I(curve, mass, d_le = D_LE, d_h = D_H, d_te = D_TE, debug = False) -> np.array:
    #leading edge
    I_LE = line_I(0, 0, curve.y3, 0, diameter=d_le)
    #hinge
    I_H = line_I(curve.y0, curve.z0, curve.y3, curve.z3, diameter=d_h)
    
    t1 = 2 * curve.z1 - curve.z0 - curve.z2 

    t2 = curve.z1**2 - curve.z1 * curve.z2 - curve.z1 * curve.z3 + curve.z0 * curve.z3 + curve.z2**2 - curve.z0 * curve.z2 

    t3 = 3 * curve.z1 - curve.z0 - 3 * curve.z2 + curve.z3 

    t_max = 0.5 if abs(t3) < 1e-6 else t1 + np.sqrt(t2) / t3

    #trailing edge
    I_TE = curve_I(curve, t_max = t_max, diameter=d_te)
    
    #membrane
    I_film = film_I(curve)
    
    if debug:
        print(I_LE)
        print(I_H)
        print(I_TE)
        print(I_film)
    
    I_origin = I_LE + I_H + I_TE + I_film

    com = (line_com(0, 0, curve.y3, 0, diameter=d_le) + line_com(curve.y0, curve.z0, curve.y3 - d_h - d_te) + curve_com(curve, diameter=d_te, t_max = t_max) + film_com(curve))/mass
    com_r_sqr = com[0]**2 + com[1]**2

    return I_origin - mass * np.array([com_r_sqr, com_r_sqr - com[0]**2, com_r_sqr - com[1]**2, 0, 0, -com[0]*com[1]])
    
def dm_curve(curve, t, rho):
    return rho * np.sqrt(curve.lower_dz(t)**2 + curve.dy(t)**2)

def bez_wing_mass(curve, d_le = D_LE, d_h = D_H, d_te = D_TE) -> float:
    rho_le = np.pi / 4 * d_le**2 * 1854.55 #kg/m
    rho_te = np.pi / 4 * d_te**2 * 1854.55 #kg/m
    rho_h = np.pi / 4 * d_h * 1854.55 #kg/m
    m_LE = curve.y3 * rho_le
    m_h = (curve.y3 - (curve.y0 + d_h + d_te)) * rho_h
    m_film = sc.integrate.quad(lambda t: -curve.z(t) * curve.dy(t), 0, 1)

    t1 = 2 * curve.z1 - curve.z0 - curve.z2 
    t2 = curve.z1**2 - curve.z1 * curve.z2 - curve.z1 * curve.z3 + curve.z0 * curve.z3 + curve.z2**2 - curve.z0 * curve.z2 
    t3 = 3 * curve.z1 - curve.z0 - 3 * curve.z2 + curve.z3 

    t_max = 0.5 if abs(t3) < 1e-6 else t1 + np.sqrt(t2) / t3

    m_te = sc.integrate.quad(lambda t: dm_curve(curve, t, rho_te), 0, t_max)

    return m_LE + m_h + m_film + m_te
