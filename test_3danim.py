import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.animation import FuncAnimation
import mpl_toolkits.mplot3d.axes3d as p3


fig = plt.figure()
ax = p3.Axes3D(fig)
w = 10
ax.set_xlim([-w, w])
ax.set_ylim([-w, w])
ax.set_zlim([-w, w])

def update(frame):

    print('updating')
    ax.clear()
    # xdata.append(frame)
    # ydata.append(np.sin(frame))
    # ln.set_data(xdata, ydata)
    ax.set_xlim(0, 2 * np.pi)
    ax.set_ylim(-1, 1)
    ax.scatter(frame,np.sin(frame),0)
    # return ln,
ax.set_xlim(0, 2*np.pi)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128))
plt.show()