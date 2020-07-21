#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.mlab import griddata


def getAverages(arr, N):
    """ Average slices of length N within the array
    Args:
        arr (list): Array to get averages from
        N (int): length of slices
    Returns:
        avgs (list): Array where each value is the mean of the slice of
            length N
    """
    avgs = []
    print(arr)

    for i in range(0, len(arr), N):
        for _ in range(N):
            #mean = np.nanmean(arr[i:i+N]) # ignore nan values
            # print("HERE!")
            # print(i)
            # print(i+N)
            print(arr[i:i+N])
            mean = np.nanmean(arr[i:i+N])
            avgs.append(mean)
    return avgs


## View contour graph
filename = "/home/amy/whoi_ws/src/min_variance_calibration/min_variance_calibration/results/results.csv"

(x, y, a, b, c) = np.loadtxt(open(filename, "rb"), delimiter=",", skiprows=1)

# Number of points per measurement
N = 1

# define grid.
xi = np.linspace(x.min(),x.max(),100)
yi = np.linspace(y.min(),y.max(),100)
print(x)
print(y)

# Average duplicate measurements ----------------------------------------------
# Get unique parameter noise/measurement noise
un_x, un_y = np.unique(x), np.unique(y)
avg_a = getAverages(a, N)

ai = griddata(x, y, avg_a, xi, yi, interp='linear')

# contour the gridded data, plotting dots at the randomly spaced data points.
CS = plt.contour(xi,yi,ai,20,linewidths=0.5,colors='k')
CS = plt.contourf(xi,yi,ai,200,cmap=plt.cm.jet)

plt.colorbar() # draw colorbar
plt.clim(0,0.5)

# plot data points.
plt.scatter(x,y,marker='o',c='b',s=5)
# plt.xlim(-2,2)
# plt.ylim(-2,2)
plt.xlabel("Parameter noise (percentage)")
plt.ylabel("Measurement noise (meters)")
# plt.title('griddata test (%d points)' % npts)
plt.show()
