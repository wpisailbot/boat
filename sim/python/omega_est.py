#!/usr/bin/python3
import numpy as np
import sys
from scipy import optimize
import scipy.stats.stats
from matplotlib import pyplot as plt
import read_replay

def Norm(t):
  while t > np.pi:
    t -= 2 * np.pi
  while t < -np.pi:
    t += 2 * np.pi
  return t

(t, x, y, vx, vy, speed, yaw, heel, pitch, omega, heading,
 leeway, sail, rudder, alphaw, pitchvar, wind_speed, true_alphaw,
 true_wind_speed, heading_cmd, rudder_mode, _, _) = read_replay.read("sep11replay.csv", False, 1000, 6000)

diffyaw = [Norm(y) for y in np.diff(yaw)]
tdiff = t[1:]
filtdiff = []
N2 = 2
for i in range(len(diffyaw)):
  mini = max(i-N2, 0)
  maxi = min(i+N2, len(diffyaw))
  filtdiff.append(np.mean(diffyaw[mini:maxi]))

#plt.plot(tdiff, filtdiff, 'b', label="Filtered change in heading (127250) / 10")
plt.plot(t, omega, 'g', label="Rate of Turn (127251) data")
plt.title("Airmar 220WX Rate-of-Turn + heading data")
plt.xlabel("Time (sec)")
plt.ylim([-0.015, 0.015])
plt.ylabel("Rate of turn (rad/s) and change in heading (tens of rad / sec)")
plt.legend(loc='upper left')
plt.twinx()
plt.plot(t, yaw, 'r', label="yaw")
plt.ylabel("Heading (rad)")
plt.legend(loc='upper right')

plt.show()
