#!/usr/bin/python3
import numpy as np

data = np.genfromtxt("race.csv")

with open("race.csv") as pts:
    for pt in pts:
        pt = pt.split(' ')
        lat = float(pt[1]) + float(pt[2]) / 60.0
        lon = float(pt[4]) + float(pt[5]) / 60.0
        print("points: {x: " + repr(lon) + ", y: " + repr(lat) + "},")
