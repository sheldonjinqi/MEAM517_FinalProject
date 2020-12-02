import matplotlib.animation as animation
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Circle

import numpy as np

fig = plt.figure()
ax = p3.Axes3D(fig)
w = 10
ax.set_xlim([-w, w])
ax.set_ylim([-w, w])
ax.set_zlim([-w, w])


x_axis = np.arange(-2, 3)
y_axis = np.arange(-2, 3)
z_axis = np.arange(-2, 3)
pointCM, = ax.plot([0], [0], [0], 'b.')
pointBLDC1, = ax.plot([0], [0], [0], 'b.')
pointBLDC2, = ax.plot([0], [0], [0], 'b.')
pointBLDC3, = ax.plot([0], [0], [0], 'b.')
pointBLDC4, = ax.plot([0], [0], [0], 'b.')
# line1, = ax.plot([0,0], [0,0], [0,0], 'b.')
# line2, = ax.plot([0,0], [0,0], [0,0], 'r.')

# ax.plot([0,0], [0,0], [0,0], 'k+')
ax.plot(x_axis, np.zeros(5), np.zeros(5), 'r--', linewidth = 0.5)
ax.plot(np.zeros(5), y_axis, np.zeros(5), 'g--', linewidth = 0.5)
ax.plot(np.zeros(5), np.zeros(5), z_axis, 'b--', linewidth = 0.5)


ax.set_xlabel('X-axis (in meters)')
ax.set_ylabel('Y-axis (in meters)')
ax.set_zlabel('Z-axis (in meters)')

# time_display = ax.text(22.0, 1.0, 39.0, "red" ,color='red', transform=ax.transAxes)
# state_display = ax.text(1.0, 1.0, 41.0, "green" ,color='green', transform=ax.transAxes)

x_bl1 = 1
x_bl2 = 0
x_bl3 = -1
x_bl4 = 0

y_bl1 = 0
y_bl2 = 1
y_bl3 = 0
y_bl4 = -1

z_bl1 = 2
z_bl2 = 2
z_bl3 = 2
z_bl4 = 2

# ax.scatter(0,0,2)
line1, = ax.plot([x_bl4, x_bl2], [y_bl4, y_bl2], [z_bl4, z_bl2], 'ko-', lw=1.5, markersize=3)
line2, = ax.plot([x_bl3, x_bl1], [y_bl3, y_bl1], [z_bl3, z_bl1], 'ko-', lw=1.5, markersize=3)
# p = Circle((x_bl1, y_bl1),  0.5, fill= False , color='b')
# ax.add_patch(p)
# art3d.pathpatch_2d_to_3d(p, z=z_bl1, zdir="z")
plt.show()