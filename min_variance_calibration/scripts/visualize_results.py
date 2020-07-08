#!/usr/bin/env python

from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
import numpy.ma as ma
from numpy.random import uniform, seed

import numpy as np


filename = "/home/amy/whoi_ws/src/min_variance_calibration/min_variance_calibration/results/results.csv"

(x, y, z) = np.loadtxt(open(filename, "rb"), delimiter=",", skiprows=1)

# z = z - y
print(x)
print(x.min())

# Save data to output file so it doesn't need re-running
# make up some randomly distributed data
# seed(1234)
npts = 200
# x = uniform(-2,2,npts)
# y = uniform(-2,2,npts)
# z = np.random.random(len(x))

# define grid.
xi = np.linspace(x.min(),x.max(),100)
yi = np.linspace(y.min(),y.max(),100)
# grid the data.
zi = griddata(x, y, z, xi, yi, interp='linear')

# contour the gridded data, plotting dots at the randomly spaced data points.
CS = plt.contour(xi,yi,zi,20,linewidths=0.5,colors='k')
CS = plt.contourf(xi,yi,zi,20,cmap=plt.cm.jet)
plt.colorbar() # draw colorbar
# plot data points.
plt.scatter(x,y,marker='o',c='b',s=5)
# plt.xlim(-2,2)
# plt.ylim(-2,2)
plt.xlabel("Parameter noise (percentage)")
plt.ylabel("Measurement noise (meters)")
plt.title('griddata test (%d points)' % npts)
plt.show()
