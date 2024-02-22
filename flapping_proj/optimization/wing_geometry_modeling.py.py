import numpy as np
import matplotlib.pyplot as plt
import scipy as sc

plt.rcParams['text.usetex'] = True

D_1 = 0.002 #m
D_2 = 0.001 #m

rho_1 = np.pi / 4 * D_1**2 * 1854.55 #kg/m
rho_2 = np.pi / 4 * D_2**2 * 1854.55 #kg/m

rho = 1.263 #kg/m**3, air density

class BezierCurve():
    x_0 = None
    y_0 = None
    x_1 = None
    y_1 = None
    x_2 = None
    y_2 = None
    x_3 = None
    y_3 = None
    
    def B_x(self, t):
        return (1 - t)**3 * self.x_0 + 3*(1 - t)**2 * t * self.x_1 + 3*(1 - t) * t**2 * self.x_2 + t**3  * self.x_3
    
    def B_y(self, t):
        return (1 - t)**3 * self.y_0 + 3*(1 - t)**2 * t * self.y_1 + 3*(1 - t) * t**2 * self.y_2 + t**3  * self.y_3
    
    def dB_x(self, t):
        return 3 * (1 - t)**2 * (self.x_1 - self.x_0) + 6 * (1 - t) * t * (self.x_2 - self.x_1) + 3 * t**2 * (self.x_3 - self.x_2)
    
    def dB_y(self, t):
        return 3 * (1 - t)**2 * (self.y_1 - self.y_0) + 6 * (1 - t) * t * (self.y_2 - self.y_1) + 3 * t**2 * (self.y_3 - self.y_2)
    
    def __init__(self, x_0, y_0, x_1, y_1, x_2, y_2, x_3, y_3):
        self.x_0 = x_0
        self.y_0 = y_0
        self.x_1 = x_1
        self.y_1 = y_1
        self.x_2 = x_2
        self.y_2 = y_2
        self.x_3 = x_3
        self.y_3 = y_3
        
class DrivingFunction():
    f = 20
    phi_max = np.pi/3
    
    def phi(self, t):
        return  self.phi_max * np.sin(2 * np.pi * self.f * t)
    
    def dphi(self, t):
        return self.phi_max * np.cos(2 * np.pi * self.f * t) * 2 * np.pi * self.f
    
    def ddphi(self, t):
        return -self.phi_max * np.sin(2 * np.pi * self.f * t) * (2 * np.pi * self.f)**2
    
    def __init__(self, f, phi_max):
        self.f = f
        self.phi_max = phi_max

def C_L(alpha):
    return 0.225 + 1.58 * np.sin(2.03 * alpha - 7.2 / 180 * np.pi)

def C_D(alpha):
    return 1.92 - 1.55 * np.cos(2.04 * alpha - 9.82 / 180 * np.pi)

def T_1(phi):
    return np.array([[np.cos(phi), -np.sin(phi), 0],[np.sin(phi), np.cos(phi), 0],[0, 0, 1]])

def T_2(psi):
    return np.array([[1, 0, 0],[0, np.cos(psi), -np.sin(psi)],[0, np.sin(psi), np.cos(psi)]])

def alpha(psi):
    return np.pi/2 - np.abs(psi)

def I_LE(R):
    return np.array([[0, 0, 0], [0, rho_1 * R**3 / 3, 0], [0, 0, rho_1 * R**3 / 3]])

def dm(t, curve):
    return rho_2 * np.sqrt(curve.dB_x(t)**2 + curve.dB_y(t)**2) #dt

def I_TE(curve): 
    def I_XXint(t, curve):
        return curve.B_y(t)**2 * dm(t, curve)
    
    def I_YYint(t, curve):
        return curve.B_x(t)**2 * dm(t, curve)
    
    def I_ZZint(t, curve):
        return (curve.B_x(t)**2 + curve.B_y(t)**2) * dm(t, curve)
    
    def I_XYint(t, curve):
        return curve.B_x(t) * curve.B_y(t) * dm(t, curve)
    
    I_xx = sc.integrate.quad(I_XXint, 0, 1, args = (curve))[0]
    I_yy = sc.integrate.quad(I_YYint, 0, 1, args = (curve))[0]
    I_zz = sc.integrate.quad(I_ZZint, 0, 1, args = (curve))[0]
    I_xy = sc.integrate.quad(I_XYint, 0, 1, args = (curve))[0]
    
    return np.array([[I_xx, -I_xy, 0], [-I_xy, I_yy, 0], [0, 0, I_zz]])


def dF_d(t, psi, dpsi, dphi, curve):
    alpha = np.pi / 2 - np.abs(psi) 
        
    V_f = -dphi * curve.B_x(t)
    V_i = -curve.B_y(t) * .25 * dpsi + np.abs(V_f) * np.tan(alpha)
    V = np.sqrt(V_f**2 + V_i**2)
    
    return -.5 * rho * (0 - curve.B_y(t)) * (C_L(alpha) * V_i + C_D(alpha) * V_f) * V * curve.dB_x(t)

def dF_T(t, psi, dpsi, dphi, curve):
    alpha = np.pi / 2 - np.abs(psi)
    V_f = -dphi * curve.B_x(t)
    V_i = -curve.B_y(t) * .25 * dpsi + np.abs(V_f) * np.tan(alpha)
    V = np.sqrt(V_f**2 + V_i**2)
    
    return .5 * rho * (0 - curve.B_y(t)) * (C_L(alpha) * V_f * np.sign(V_f * psi) - C_D(alpha) * V_i) * V * curve.dB_x(t)

def F_AERO(psi, dpsi, dphi, curve): #returns the aerodynamic forces in the LE frame
    F_d = sc.integrate.quad(dF_d, 0, 1, args = (psi, dpsi, dphi, curve))[0]
    F_T = sc.integrate.quad(dF_T, 0, 1, args = (psi, dpsi, dphi, curve))[0]
    
    return np.array([0, F_d, F_T]).transpose()

def TAU_AERO(psi, dpsi, dphi, curve): #gets the aerodynamic torque in an instant for a geometry in the TE frame
    def dtau_psi(t, psi, dpsi, dphi, curve):
        return .25 * (0 - curve.B_y(t)) * (np.sin(psi) * dF_T(t, psi, dpsi, dphi, curve) + np.cos(psi) * dF_d(t, psi, dpsi, dphi, curve))
    
    def dtau_phi(t, psi, dpsi, dphi, curve):
        return curve.B_x(t) * dF_d(t, psi, dpsi, dphi, curve)
    
    tau_psi = sc.integrate.quad(dtau_psi, 0, 1, args = (psi, dpsi, dphi, curve))[0]
    tau_phi = sc.integrate.quad(dtau_phi, 0, 1, args = (psi, dpsi, dphi, curve))[0]
    
    return T_2(psi) @ (np.array([tau_psi, 0, tau_phi]).transpose())

def TAU_EXT(R, psi, dpsi, phi, dphi, ddphi, I_t, tau_phi):
    I_xxt = I_t[0][0]
    I_yyt = I_t[1][1]
    I_zzt = I_t[2][2]
    I_xyt = -I_t[0][1]
    
    t1 = -I_xxt * np.tan(psi) * dphi * dpsi
    t2 = I_xyt * np.sin(psi)**2 * dphi**2 / np.cos(psi)
    t3 = - I_xyt * dpsi**2 / np.cos(psi)
    t4 = I_yyt * np.tan(psi) * dphi * dpsi
    t5 = I_zzt * np.tan(psi) * dphi * dpsi
    t6 = I_zzt * ddphi
    t7 = R**3 * rho_1 * ddphi / 3
    
    tau_ext = t1 + t2 + t3 + t4 + t5 + t6 + t7 - tau_phi
    
    return(tau_ext)


def ddpsiODE(t, q, R, curve, phiF, I_t, K):
    # psi_max = (0 - curve.B_y(1) - D_1 / 2) / D_1
    # print(f"t = {t}")
    
    psi = q[0]
    dpsi = q[1]
    I_t = I_TE(curve)
    I_xxt = I_t[0][0]
    I_yyt = I_t[1][1]
    I_zzt = I_t[2][2]
    I_xyt = -I_t[0][1]
    
    tau_psi = 0 #TAU_AERO(psi, dpsi, phiF.dphi(t), curve)[0]
    # print(tau_psi)
    
    tau_c = K * psi
    # if np.abs(psi) > psi_max:
    #     tau_c = np.sign(psi) * (np.abs(psi) - psi_max) * K
    
    ddpsi = (I_xyt * np.sin(psi) * phiF.ddphi(t) + .5 * I_yyt * np.sin(2 * psi) * phiF.dphi(t)**2 - .5 * I_zzt * np.sin(2 * psi) * phiF.dphi(t)**2 + tau_psi - tau_c) / I_xxt 
    
    # ddpsi *= 1 - (psi / psi_max)**2

    
    return np.array([dpsi, ddpsi])

def optCost(x):
    # print(f"Running {x}")
    y_max, x_1, y_1, x_2, y_2, R, K = x
    
    # print(f"Running with: {x}")
    #generating the parameters for the physical system
    f = 20
    phiF = DrivingFunction(f, np.pi/3)
    
    curve = BezierCurve(.008, y_max, x_1, y_1, x_2, y_2, R, y_max)
    psi_max = (0 - curve.B_y(1) - D_1 / 2) / D_1
    I_t = I_TE(curve)
    
    #running the dynamics. What the simulator would do
    sol = sc.integrate.solve_ivp(ddpsiODE, (0, 1), [0, 0], args = (R, curve, phiF, I_t, K), max_step = 1/f/10, method = "BDF")
    
    #exceptions for if the solver fails
    if sol.success == False:
        return np.inf
    if np.amax(sol.y[0]) > psi_max:
        return np.inf
    elif np.amin(sol.y[0]) < -psi_max:
        return np.inf
    
    # ddpsi = np.zeros_like(sol.t)
    
    #getting force data
    lift = np.zeros_like(sol.t)
    tau_ext = np.zeros_like(sol.t)
    
    for i in range(lift.size):
        # ddpsi[i] = ddpsiODE(sol.t[i], sol.y[:, i], R, curve, phiF, I_t, K)[1]
        
        lift[i] = F_AERO(sol.y[0, i], sol.y[1, i], phiF.dphi(sol.t[i]), curve)[2]
        tau_phi = TAU_AERO(sol.y[0, i], sol.y[1, i], phiF.dphi(sol.t[i]), curve)[2]
        tau_ext[i] = TAU_EXT(R, sol.y[0, i], sol.y[1, i], phiF.phi(sol.t[i]), phiF.dphi(sol.t[i]), phiF.ddphi(sol.t[i]), I_t, tau_phi)
    
    #mass of the wing
    m = rho_1 * R + sc.integrate.quad(dm, 0, 1, args = (curve))[0]
    
    #finding the last few flap cycles
    n = 4 #number of flaps
    last_flaps = np.where(sol.t > np.amax(sol.t) - n / f)
    
    #  .plot(sol.t[last_flaps], tau_ext[last_flaps] * phiF.dphi(sol.t[last_flaps]))
    P = tau_ext[last_flaps] * phiF.dphi(sol.t[last_flaps])
    # plt.plot(sol.t[last_flaps], phiF.dphi(sol.t[last_flaps]))
    
    #root mean square power consumed (torque * angular velocity at wing root)
    RMSP = np.sqrt(sc.integrate.simpson(P**2, x = sol.t[last_flaps]) * f / n)
    #average lift for n cycles
    F_l = sc.integrate.simpson(lift[last_flaps], x = sol.t[last_flaps]) * f / n
    
    cost = m * RMSP / F_l
    
    #negative costs are very bad (caused by negative lift)
    if cost < 0:
        return np.inf
    
    return cost
    
# print(optCost([-.0025, .05, -.2, .1, -.2, .1, .07]))

if __name__ == "__main__":
    #setting system bounds
    bounds = [(-np.pi / 4 * D_1 - D_1 / 2, -.0015), (0.008, 1), (-.2, -.0015), (0.008, 1), (-.2, -.0015), (.008, .1), (0, .5)]
    
    #nonlinear constraints
    A = np.array([[0, 1, 0, -1, 0, 0, 0], [0, 0, 0, 1, 0, -1, 0], [-1, 0, 1, 0, 0, 0, 0], [-1, 0, 0, 0, 1, 0, 0]])
    
    ub = np.zeros(4)
    lc = sc.optimize.LinearConstraint(A, ub = ub)
    
    #y_max, x_1, y_1, x_2, y_2, R, K
    x0 = [-2.523e-03,  8.325e-02, -1.074e-01,  3.314e-01, -7.185e-01,
          3.323e-01,  .5]
    
    #optimization with optCost as the cost function
    res = sc.optimize.differential_evolution(optCost, bounds, constraints = lc, disp = True, maxiter = 300, updating = "deferred", workers = 2, mutation=(.5, 1.5), polish = False, x0 = x0)
    print(res)
    
    # line = BezierCurve(.008, x0[0], x0[1], x0[2], x0[3], x0[4], x0[-2], x0[0]) 
    
    
    # t = np.linspace(0, 1)
    # plt.plot(line.B_x(t), line.B_y(t))
    
    # f = 20
    # amp = np.pi/3
    # phiF = DrivingFunction(f, amp)
    
    # I_t = I_TE(line)
    
    # sol = sc.integrate.solve_ivp(ddpsiODE, (0, 2), [-np.pi / 6, 0], args = (x0[-2], line, phiF, I_t, x0[-1]), max_step = 1/f/20, method = "BDF")
    
    # # print(sol.y[:, 0])
    # t_range = np.where(sol.t > 0 - 4/f)
    # plt.figure()
    # plt.plot(sol.t[t_range], sol.y[0][t_range])
    # plt.plot(sol.t[t_range], phiF.phi(sol.t)[t_range])
    
    # ddpsi = np.ones_like(sol.t)
    # thrust = np.ones_like(sol.t)
    # drag = np.ones_like(sol.t)
    # for i in range(sol.t.size):
    #     ddpsi[i] = ddpsiODE(sol.t[i], sol.y[:, i], x0[-1], line, phiF, I_t, x0[-1])[1]
    #     thrust[i] = F_AERO(sol.y[0, i], sol.y[1, i], phiF.dphi(sol.t[i]), line)[2]
    #     drag[i] = F_AERO(sol.y[0, i], sol.y[1, i], phiF.dphi(sol.t[i]), line)[1]
    
    # plt.figure()
    # plt.plot(sol.t[t_range], thrust[t_range])
    # plt.plot(sol.t[t_range], drag[t_range])
  