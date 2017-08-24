#!/usr/bin/python3
from haversine import haversine
import numpy as np
import sys
# lat, lon
data = np.genfromtxt(sys.argv[1])
def getrelm(pt, relm):
  eps = 1e-4
  latperm =  0.0000089932
  lonperm = 0.00001154
  return (pt[0] + relm[1, 0] * latperm, pt[1] + relm[0, 0] * lonperm)

def printpts(pts):
  string = "["
  for p in pts:
    string += '{"x":' + repr(p[1]) + ',"y":' + repr(p[0]) + '},'
  string = string[:-1] + "]"
  print(string)

def printpba(pts):
  for p in pts:
    print("points: {x: " + repr(p[1]) + ", y: " + repr(p[0]) + "},")

def rotfromdiff(p1, p2):
  diff = (p2[0] - p1[0], p2[1] - p1[1])
  ang = np.arctan2(diff[0], diff[1])
  sa = np.sin(ang)
  ca = np.cos(ang)
  R = np.matrix([[ca, -sa],[sa, ca]])
  return R

finalpts = [data[0, :]]
#diff = (data[1, 0] - data[0, 0], data[1, 1] - data[0, 1])
R0 = rotfromdiff(data[1, :], data[3, :])
R1 = rotfromdiff(data[2, :], data[4, :])
#latprop = diff[0] / (diff[0] + diff[1])
#lonprop = diff[1] / (diff[0] + diff[1])
finalpts.append(getrelm(data[1, :], R0 * np.matrix([[-10],[-10]])))
finalpts.append(getrelm(data[2, :], R1 * np.matrix([[-10],[-10]])))
finalpts.append(getrelm(data[3, :], R0 * np.matrix([[10],[10]])))
finalpts.append(getrelm(data[4, :], R1 * np.matrix([[10],[10]])))
finalpts.append(data[1, :])
finalpts.append(data[2, :])
finalpts.append(data[3, :])
finalpts.append(data[4, :])
#finalpts.append(np.mean(data[(0,1,2,3), :], axis=0))

#####TODO######
# Figure out how to repeat waypoints

printpts(finalpts)
printpba(finalpts)
