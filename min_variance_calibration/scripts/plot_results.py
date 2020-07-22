#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.mlab import griddata
import pandas as pd
from scipy import stats

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
filename = "/home/amy/whoi_ws/src/min_variance_calibration/min_variance_calibration/results/combined.csv"

df = pd.read_csv(filename, delimiter=",")
# remove outliers
df = df[np.abs(stats.zscore(df['param_noise'])) < 3]
# restricting measurement noise to 0.5m (50 cm)
# df = df[df['measurement_noise'] < 0.5]
# df = pd.DataFrame(np.random.randn(100, 3))
# df = df[(np.abs(stats.zscore(df)) < 3).all(axis=1)]
# print(df)

print(df.columns)
x = df['param_noise']
y = df['measurement_noise']
a = df['accuracy']
b = df['precision']
c = df['output_variance']
plt.plot(a)
plt.plot(b)
plt.plot(c)

plt.show()

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
avg_a = getAverages(b, N)

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
