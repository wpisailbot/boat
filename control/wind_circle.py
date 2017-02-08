#!/usr/bin/python3

from matplotlib import pyplot as plt
import numpy as np

def wind_circle(theta):
  print(theta)
  while (theta > np.pi):
    theta -= 2 * np.pi
  while (theta < -np.pi):
    theta += 2 * np.pi

  theta = abs(theta) # Plot is symmetrical
  irons = np.pi / 6.
  beam_speed = 1.
  beam_loc = np.pi / 2.
  broad_loc = 3. * np.pi / 4.
  down_speed = .5
  if theta < irons:
    return 0.
  elif theta < beam_loc:
    x = theta - irons
    b = beam_loc - irons
    w = beam_speed / b ** 2 * (b * 2 - x) * x
    return w
  elif theta < broad_loc:
    return beam_speed
  elif theta < np.pi:
    x = theta - broad_loc
    return beam_speed + x * (down_speed - beam_speed) / (np.pi - broad_loc)

#ax = plt.subplot(111, projection='polar')
ax = plt.subplot(111)

theta = np.arange(0, 2*np.pi, 0.01)
wind = np.vectorize(wind_circle, otypes=[np.float])

ax.plot(theta, wind(theta))
#ax.set_rmax(1.2)
ax.grid(True)
plt.show()
