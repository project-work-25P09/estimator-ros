#!/usr/bin/env python3
import numpy as np

def read_csv(fp):
    with open(fp, "r") as f:
        return [[float(y) for y in x.split(',') if y] for x in f.read().split('\n') if x]

real_dist_m = 1.0

x_fp = './data/calibration/x.csv'
y_fp = './data/calibration/y.csv'

x = np.array([a[1:3] for a in read_csv(x_fp)])
y = np.array([a[1:3] for a in read_csv(y_fp)])

x_dist = np.hypot(np.sum(x[:,0]), np.sum(x[:,1]))
y_dist = np.hypot(np.sum(y[:,0]), np.sum(y[:,1]))

x_coeff = float(real_dist_m/x_dist)
y_coeff = float(real_dist_m/y_dist)

print(f"{x_coeff=:0.12f}, {y_coeff=:0.12f}")
