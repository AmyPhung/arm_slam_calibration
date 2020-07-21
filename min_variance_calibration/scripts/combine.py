#!/usr/bin/env python

import os
import glob
import pandas as pd


path = "/home/amy/whoi_ws/src/min_variance_calibration/min_variance_calibration/results/to_combine/"
extension = "csv"
os.chdir(path)
files = glob.glob('*.{}'.format(extension))

print(files)
li = []

for file in files:
    df = pd.read_csv(path + file, index_col=None, header=0)
    li.append(df)

frame = pd.concat(li, axis=0, ignore_index=True)
frame.to_csv("combined.csv")
print(frame)
