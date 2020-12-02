import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation
from crazyflie_params import quad_params
import time
from quad_animate import animate
import matplotlib.pyplot as plt

def quad_simulate(initial_state, quadrotor, controller, trajectory, t_final):
    " simulate the quadrotor"

    return None


class Quadrotor():
    ### define the dynamics model of the hybrid system ###

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

        self.inertia_mat = np.diag([self.Ixx, self.Iyy, self.Izz])
        self.g = 9.81  # m/s^2
        self.load = 0.01  # kg, mass of the load
        self.l = 2.0  # m, length of the cable
        self.e3 = np.array([0, 0, 1])

    # s = [x_L, x_L_dot, p, w, R, omega]
    # u1 is the input force
    # u2 is the input momentum

    def unpack_state(self, state):
        x_L = state[0:3]
        x_L_dot = state[3:6]
        p = state[6:9]
        w = state[9:12]
        R = state[12:16]
        omega = state[16:]

        return x_L, x_L_dot, p, w, R, omega



    def _s_dot_fn(self, t, s, F, M, mode=0):

        # Quadrotor with load dynamics
        if mode == 0:
            x_L, x_L_dot, p, w, R, omega = self.unpack_state(s)  # unpack the states
            r = Rotation.from_quat(R)

            R_mat = r.as_matrix()
            p_dot = np.cross(w, p)
            w_dot = -self.hat(p) @ (F * (R_mat @ self.e3)) / (self.l * self.load)  # f is thrust
            R_dot = R_mat @ self.hat(omega)
            R_dot_test = quat_dot(R,omega)
            omega_dot = np.linalg.inv(self.inertia_mat) @ (
                        M - self.hat(omega) @ (self.inertia_mat @ omega))  # M is moment
            x_L_ddot = (p @ (F * R_mat @ self.e3) - self.load * self.l * (p_dot @ p_dot)) * p / (
                        self.load + self.mass) - self.g * self.e3


        # Quadrotor-only dynamics, tension=0 for the cable
        else:
            x_L, x_L_dot, x_Q, x_Q_dot, R, omega = s

            x_L_ddot = -self.g * self.e3
            x_Q_ddot = (F / self.load) * (R @ self.e3) - self.g * self.e3
            R_dot = R @ self.hat(omega)
            omega_dot = np.linalg.inv(self.inertia_mat) @ (M - self.hat(omega) @ (self.inertia_mat @ omega))

        # print(R_dot_test)
        # R_dot = Rotation.from_matrix(R_dot)

        # R_dot_quat = R_dot.as_quat()
        state_dot = np.zeros((19,))
        state_dot[0:3]   = x_L_dot
        state_dot[3:6]   = x_L_ddot
        state_dot[6:9]  = p_dot
        state_dot[9:12] = w_dot
        state_dot[12:16] = R_dot_test
        state_dot[16:] = omega_dot


        return state_dot


    def hat(self, x):
        result = np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])
        return result

    def step(self, state, F, M, t_step):

        def s_dot_fn(t, s):
            return self._s_dot_fn(t, s, F, M)
        sol = solve_ivp(s_dot_fn, (0, t_step), state, first_step=t_step)

        s = sol.y[:, -1]

        return s


def simulate(initial_state, quadrotor, controller=0, trajectory=0, t_final=5):
    t0 = 0.0
    n_points = 1000
    t_step = 1e-3

    t = [t0]
    x = [initial_state]
    u = [np.zeros((4,))]
    start_time = time.time()
    while t[-1] < t_final:
        current_t = t[-1]
        current_x = x[-1]
        current_u = np.zeros((4,))  # using zero inputs for now
        next_x = quadrotor.step(current_x, current_u[0], current_u[1:], t_step)
        # print('current load pos', next_x[:3])
        x.append(next_x)
        t.append(t[-1] + t_step)
    print("--- %s seconds ---" % (time.time() - start_time))
    return x

def quat_dot(quat, omega):
    """
    Parameters:
        quat, [i,j,k,w]
        omega, angular velocity of body in body axes

    Returns
        duat_dot, [i,j,k,w]

    """
    # Adapted from "Quaternions And Dynamics" by Basile Graf.
    (q0, q1, q2, q3) = (quat[0], quat[1], quat[2], quat[3])
    G = np.array([[ q3,  q2, -q1, -q0],
                  [-q2,  q3,  q0, -q1],
                  [ q1, -q0,  q3, -q2]])
    quat_dot = 0.5 * G.T @ omega
    # Augment to maintain unit quaternion.
    quat_err = np.sum(quat**2) - 1
    quat_err_grad = 2 * quat
    quat_dot = quat_dot - quat_err * quat_err_grad
    return quat_dot

if __name__ == '__main__':
    quad = Quadrotor(quad_params)
    initial_rotation = Rotation.from_matrix(np.eye(3))
    # initial_rotation = Rotation.from_matrix(np.array([[1,0,0],[0,0,-1],[0,1,0]]))

    initial_rotation_quat = initial_rotation.as_quat()
    initial_state = np.zeros((19,))
    initial_state[12:16] = initial_rotation_quat
    initial_state[6:9] = np.array([0, -1/np.sqrt(2), -1/np.sqrt(2)])

    x = simulate(initial_state, quad)
    x = np.asarray(x)
    ani_test = animate(5,x,quad,n_frame=60)
    plt.show()
