import numpy as np
import json
from optimization.top_level import json_config_path

# Determines the approximate flapping amplitude assuming the wing does not 
# pitch and that there are no aerodynamic forces. The motor wing system can be approximated
# by a mass-spring-damper system excluding the aerodynamics and pitching, so there is a known steady
# state solution. 

# This wing system can be described as I_zz * ddphi + k_t * phi = tau_in
# where I_zz is the moment of inertia about the stroke axis, k_t is the stroke
# axis spring constant, tau_in is the input torque from the motor, and (d)phi is the current
# angle about the stroke axis (and it's derivatives). 

# For the config file, 
# I_zz = wing[inertia][izz] + wing[mass] * wing[com][2]**2
# k_t = stroke_joint[spring_stiffness]

# tau_in is a function of the motor properties and current state

# tau_in = N_g * i_a * k_a
# U sin(2 * pi * f * t) = i_a * R_a + k_a * N_g * dphi
# N_g = inputs[motor_gear_ratio] * inputs[external_gear_ratio]
# i_a = motor armature current
# k_a = inputs[motor_torque_constant]
# U = inputs[max_voltage]
# f = inputs[frequency]

# Overall, this system can be put in to the form
# m * ddx + c * dx + k * x = f(t), which is a basic forced mass-spring-damper
# here, f(t)  = F * sin(omega * t)

def flapping_amp():
    with open(json_config_path, 'r') as json_file:
        config = json.load(json_file)

    n_g = config["inputs"]["motor_gear_ratio"] * \
        config["inputs"]["external_gear_ratio"]
    r_a = config["inputs"]["motor_resistance"]
    k_a = config["inputs"]["motor_torque_constant"]
    f = config["inputs"]["frequency"]
    u_max = config["inputs"]["max_voltage"]  # U in the motor equations

    m = config["wing"]["inertia"]["izz"] + \
        config["wing"]["mass"] * config["wing"]["com"][2]**2
    k = config["stroke_joint"]["spring_stiffness"]
    c = n_g**2 * k_a**2 / r_a
    forced_amp = n_g * k_a * u_max / r_a  # F in f(t)
    omega = 2 * np.pi * f

    omega_n = np.sqrt(k / m)
    damping_ratio = c / (2 * np.sqrt(m * k))

    A = forced_amp / m * ((omega_n**2 - omega**2) /
                        ((omega_n**2 - omega**2)**2 + (2*damping_ratio*omega*omega_n)**2))
    B = forced_amp / m * ((2*damping_ratio*omega*omega_n) /
                        ((omega_n**2 - omega**2)**2 + (2*damping_ratio*omega*omega_n)**2))

    response_amp = np.sqrt(A**2 + B**2)
    print(response_amp)
    
if __name__ == "__main__":
    flapping_amp()
