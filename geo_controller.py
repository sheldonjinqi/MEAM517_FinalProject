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

        # Controller gains
        self.k_x = np.diag((1,1,1))
        self.k_v = np.diag((1,1,1))

    def compute_input(self,t,state,flat_output):

        # Load position controller
        k_x = self.k_x
        k_v = self.k_v

        # load the data for mode=1, nonzero tension
        x_L = state[0:3]
        x_L_dot = state[3:6]
        q = state[6:9]
        w = state[9:12]
        R = state[12:16]
        omega = state[16:]

        q_dot = np.cross(w,q)

        x_L_d = flat_output['x_d']
        x_L_dot_d = flat_output['x_dot_d']
        x_L_ddot_d = flat_output['x_L_ddot_d']
        q_dot_d = flat_output['q_dot_d']
        q_ddot_d = flat_output['q_ddot_d']
        yaw_d = flat_output['yaw_d']
        yaw_dot_d = flat_output['yaw_dot_d']

        e_x = x_L - x_L_d
        e_v = x_L_dot - x_L_dot_d
        A = -k_x @ e_x - k_v @ e_v +  (self.m_q + self.m_L)*(x_L_ddot_d + self.g * self.e3) + self.m_q*self.l*(q_dot.dot(q_dot))*q
        q_d = - A / np.linalg.norm(A) # desired unit vector q from Quad to Load
        q_dot_d = np.cross(w,q_d)
        e_q = hat(q)@hat(q)@q_d
        e_q_dot = q_dot - np.cross(np.cross(q_d,q_dot_d),q)
        # Load attitude controller
        F_n = -(q_d.dot(q))*q # double check
        F_n = A.dot(q_d)@q_d
        F_pd = -k_q @ e_q - k_w @ e_q_dot
        F_ff = m_q * l *(q.dot(np.cross(q_d,q_dot_d))) + np.cross(np.cross(q_d,q_ddot_d),q)

        F_d = F_n - F_pd - F_ff # desired force

        b3_d = F_d / np.linalg.norm(F_d)
        b1_d = - (np.cross(b3_d,np.cross(b3_d, yaw_d))) / (np.linalg.norm(np.cross(b3_d, yaw_d)))
        b2_d = np.cross(b3_d,b1_d)
        R_d = np.concatenate((b1_d,b2_d,b3_d))
        R_dot_d = R @ hat(omega)
        omega_d = np.array([0,0,yaw_dot_d]) # first two are 0 in practice but can be calculated

        thrust = F.dot(R@e3)
        w_dot = -self.hat(p) @ (thrust * (R @ self.e3)) / (self.l * self.load)
        w_d = R_d.T @ (R_d@hat(omega)) # desiredangular velocity of laod


        M_d = -1/(epsilon**2)*(k_r @ e_R) - 1/epsilon * k_omega @ e_omega + np.cross(omega,I@omega)
                -  I @ (hat(omega)@R.T@R_d@omega_d-R.T@R_d@omega_dot_d)

        # Quad attitude controller


        return None

