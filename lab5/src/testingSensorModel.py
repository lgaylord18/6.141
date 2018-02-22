import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from mpl_toolkits.mplot3d import Axes3D

table_width = 200
(x,y) = np.meshgrid(np.linspace(0,table_width,table_width+1),np.linspace(0,table_width,table_width+1))
z = norm.pdf(x, y, 10)

z += .005
for row in xrange(table_width+1):
	z[row][0:row] += .02 - .02*np.arange(row)/row
z[:,-3:] = .075

for i in range(len(z)):
	z[i]= z[i]/sum(z[i])

z = z.T


#This block plots where the car is in its environment
fig = plt.figure()
ax = fig.gca(projection='3d')
surf = ax.plot_surface(x, y, z, cmap="bone", rstride=2, cstride=2, linewidth=0, antialiased=True)
ax.text2D(0.05, 0.95, "Precomputed Sensor Model", transform=ax.transAxes)
ax.set_xlabel('Ground truth distance (in px)')
ax.set_ylabel('Measured Distance (in px)')
ax.set_zlabel('P(Measured Distance | Ground Truth)')
plt.show('hold')
