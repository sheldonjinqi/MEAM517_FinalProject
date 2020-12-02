import numpy as np
from scipy.spatial.transform import Rotation

class GeoControl(object):
    # Quadrotor physical parameters.
    def __init__(self, quad_params):
        self.mass = quad_params['mass']  # kg
        self.Ixx = quad_params['Ixx']  # kg*m^2
        self.Iyy = quad_params['Iyy']  # kg*m^2
        self.Izz = quad_params['Izz']  # kg*m^2
        self.arm_length = quad_params['arm_length']  # meters
        self.rotor_speed_min = quad_params['rotor_speed_min']  # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max']  # rad/s
        self.k_thrust = quad_params['k_thrust']  # N/(rad/s)**2
        self.k_drag = quad_params['k_drag']  # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.g = 9.81  # m/s^2
        self.gamma = self.k_drag / self.k_thrust

    def compute_input(self,t,state,flat_output):

        # Load position controller
        A = -k_x @ e_x - k_v @ e_v +  (self.m_q + self.m_L)*(x_L_ddot_d + self.g * self.e3) + self.m_q*self.l*(q_dot.dot(q_dot))*q
        q_d = - A / np.linalg.norm(A) # desired unit vector q from Quad to Load


        # Load attitude controller
        F_n = -(q_d.dot(q))*q 

        # Quad attitude controller


        return None

