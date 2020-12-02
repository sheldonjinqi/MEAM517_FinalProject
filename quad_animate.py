import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import mpl_toolkits.mplot3d.axes3d as p3
from scipy.spatial.transform import Rotation

def unpack_state(state):
    x_L = state[:,0:3]
    x_L_dot = state[:,3:6]
    p = state[:,6:9]
    w = state[:,9:12]
    R = state[:,12:16]
    omega = state[:,16:]

    return x_L, x_L_dot, p, w, R, omega

def animate(t_f,state,quad,n_frame = 30):

    n_samples = 1000
    t_samples = np.linspace(0.0,t_f,n_samples)

    fig = plt.figure()
    ax = p3.Axes3D(fig)
    w = 10
    ax.set_xlim([-w, w])
    ax.set_ylim([-w, w])
    ax.set_zlim([-w, w])

    x_L, x_L_dot, p, w, R, omega = unpack_state(state)

    frame_idx = [round(x) for x in np.linspace(0,x_L.shape[0]-1,n_frame).tolist()]
    state_anim = np.zeros((n_frame,19))
    for i in range(n_frame):
        state_anim[i] = state[frame_idx[i]]

    x_L = state_anim[:,:3]
    p = state_anim[:,6:9]
    R = state_anim[:,12:16]
    print('animating')

    load, = ax.plot([0,0], [0,0], [0,0], 'b.')
    com, = ax.plot([0,0], [0,0], [0,0], 'b.')
    propeller_1, = ax.plot([0,0], [0,0], [0,0], 'b.')
    propeller_2, = ax.plot([0,0], [0,0], [0,0], 'b.')

    def update(frame):
        nonlocal load, com, propeller_1, propeller_2
        load.remove()
        com.remove()
        propeller_1.remove()
        propeller_2.remove()
        x_L_frame = x_L[frame]
        p_frame = p[frame] * quad.l
        R_frame = R[frame] # represented using quaternion
        R_ = Rotation.from_quat(R_frame)
        R_matrix = R_.as_matrix()
        x_Q_frame = x_L_frame - p_frame
        theta = np.linspace(0,2*np.pi,4,endpoint= False)
        rotor_position = np.zeros((4,3))
        rotor_position[:,0] = quad.arm_length * np.cos(theta)
        rotor_position[:,1] = quad.arm_length * np.sin(theta)

        rotor_position_I = R_matrix @ rotor_position.T + x_Q_frame.reshape(-1,1) #rotor position in world frame
        rotor_1, rotor_2, rotor_3, rotor_4 = rotor_position_I.T


        #plot the COM of UAV and load position
        com, = ax.plot([x_L_frame[0],x_Q_frame[0]],[x_L_frame[1],x_Q_frame[1]],[x_L_frame[2],x_Q_frame[2]],color='C0')
        #visualize the UAV
        propeller_1, = ax.plot([rotor_1[0], rotor_3[0]], [rotor_1[1], rotor_3[1]], [rotor_1[2], rotor_3[2]], 'ko-', color='C0')
        propeller_2, = ax.plot([rotor_2[0], rotor_4[0]], [rotor_2[1], rotor_4[1]], [rotor_2[2], rotor_4[2]], 'ko-', color='C0')
        #visualize the load as a sphere
        load = ax.scatter(x_L_frame[0],x_L_frame[1],x_L_frame[2],color='r')


    ani = FuncAnimation(fig=fig,func=update,frames=n_frame,blit=False, repeat=False)
    return ani
if __name__ == '__main__':
    x_L_frame = np.zeros((3,))
    p_frame = np.array([0,0,-5])
    R_frame = np.eye(3)
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    w = 10
    ax.set_xlim([-w, w])
    ax.set_ylim([-w, w])
    ax.set_zlim([-w, w])
    x_Q_frame = x_L_frame - p_frame
    theta = np.linspace(0, 2 * np.pi, 4, endpoint=False)
    rotor_position = np.zeros((4, 3))
    rotor_position[:, 0] = 1 * np.cos(theta)
    rotor_position[:, 1] = 1 * np.sin(theta)
    rotor_position_I = R_frame @ rotor_position.T + x_Q_frame.reshape(-1,1)  # rotor position in world frame
    rotor_1, rotor_2, rotor_3, rotor_4 = rotor_position_I.T

    # plot the COM of UAV and load position
    ax.plot([x_L_frame[0], x_Q_frame[0]], [x_L_frame[1], x_Q_frame[1]], [x_L_frame[2], x_Q_frame[2]])
    # visualize the UAV
    ax.plot([rotor_1[0], rotor_3[0]], [rotor_1[1], rotor_3[1]], [rotor_1[2], rotor_3[2]],'ko-',color='b')
    ax.plot([rotor_2[0], rotor_4[0]], [rotor_2[1], rotor_4[1]], [rotor_2[2], rotor_4[2]],'ko-',color='b')
    # visualize the load as a sphere
    ax.scatter(x_L_frame[0], x_L_frame[1], x_L_frame[2],color='r')
    plt.show()

