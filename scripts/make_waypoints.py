#!/usr/bin/python3
import numpy as np
import sys

if len(sys.argv) < 1:
  print('Need input file')

pts = np.genfromtxt(sys.argv[1], delimiter=',')

for pt in pts:
  print("points: { x: " + repr(pt[1]) + ", y: " + repr(pt[0]) + "},")
