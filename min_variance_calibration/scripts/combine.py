#!/usr/bin/env python

import os
import glob
import pandas as pd
import numpy as np

path = "/home/amy/whoi_ws/src/min_variance_calibration/min_variance_calibration/results/temp/"
extension = "csv"
os.chdir(path)
files = glob.glob('*.{}'.format(extension))

print(files)
li = []

for file in files:
    print(file)
    # (x, y, a, b, c) =
    data = np.loadtxt(open(path + file, "rb"), delimiter=",", skiprows=1)

    df = pd.DataFrame(data, index=['param_noise', 'measurement_noise', 'accuracy', 'precision', 'output_variance']).T
    print(df)
    li.append(df)

frame = pd.concat(li, axis=0, ignore_index=True)
frame.to_csv("combined.csv")
print(frame)
