"""
Physical parameters of the laboratory Crazyflie quadrotors.
Additional sources:
    https://bitcraze.io/2015/02/measuring-propeller-rpm-part-3
    https://wiki.bitcraze.io/misc:investigations:thrust
    https://commons.erau.edu/cgi/viewcontent.cgi?article=2057&context=publication
Notes:
    k_thrust is inferred from 14.5 g thrust at 2500 rad/s
    k_drag is mostly made up
"""

quad_params = {
    'mass': 0.030,   # kg
    'Ixx':  1.43e-5, # kg*m^2
    'Iyy':  1.43e-5, # kg*m^2
    'Izz':  2.89e-5, # kg*m^2
    'arm_length': 1, # meters #0.046 for crazyflies
    'rotor_speed_min': 0,    # rad/s
    'rotor_speed_max': 2500, # rad/s
    'k_thrust': 2.3e-08, # N/(rad/s)**2
    'k_drag':   7.8e-11, # Nm/(rad/s)**2
}
