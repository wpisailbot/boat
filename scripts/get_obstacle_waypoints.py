#!/usr/bin/python3
from haversine import haversine
import numpy as np
import sys
# lat, lon
data = np.genfromtxt(sys.argv[1])
def getrelm(pt, relm):
  eps = 1e-4
  latperm =  0.0000089932
  lonperm = 0.0000089932
  return (pt[0] + relm[1, 0] * latperm, pt[1] + relm[0, 0] * lonperm)

def printpts(pts):
  print("[")
  string = ""
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

printpts(data)
finalpts = [data[0, :]]
#diff = (data[1, 0] - data[0, 0], data[1, 1] - data[0, 1])
R = rotfromdiff(data[0, :], data[1, :])
#latprop = diff[0] / (diff[0] + diff[1])
#lonprop = diff[1] / (diff[0] + diff[1])
pt2 = data[1, :]
round1 = getrelm(pt2, R * np.matrix([[5],[-5]]))
round2 = getrelm(pt2, R * np.matrix([[5],[5]]))
finalpts += [round1, round2]
finalpts.append(data[2, :])
printpts(finalpts)
printpba(finalpts)
