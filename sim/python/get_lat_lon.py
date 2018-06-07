#!/usr/bin/python3
import numpy as np
data = np.genfromtxt("sep11replay.csv", delimiter=',')
print(data)
out = data[:, (13, 7, 8)]
out[:, 0] *= 1e-4
np.savetxt("/tmp/latlon.csv", out, header='Seconds since UTC midnight,lat(radians),lon(radians)', delimiter=',')
