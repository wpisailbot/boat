#!/usr/bin/python3

from matplotlib import pyplot as plt
import numpy as np
import sys

name = "/tmp/foobar"
if len(sys.argv) > 1:
  name = sys.argv[1]
data = np.genfromtxt(name, delimiter=",")
plt.plot(data[:, 1], data[:, 2], 'o')
plt.figure()
plt.plot(data[:, 0], data[:, 3])
plt.title('Velocity')
plt.figure()
plt.plot(data[:, 0], data[:, 4])
plt.title('Theta')
plt.figure()
plt.plot(data[:, 0], data[:, 5])
plt.title('U')
plt.show()
