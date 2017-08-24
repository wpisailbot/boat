#!/usr/bin/python3
import numpy as np
import sys
from matplotlib import pyplot as plt

def norm_theta(theta):
  while (theta > np.pi):
    theta -= 2 * np.pi
  while (theta < -np.pi):
    theta += 2 * np.pi
  return theta

data = np.genfromtxt("endurance_replay.csv", delimiter=',')[1:]
x = []
y = []
vx = []
vy = []
t = []
yaw = []
heel = []
heading = []
leeway = []
sail = []
rudder = []
alphaw = []
wind_speed = []
true_alphaw = []
true_wind_speed = []
for row in data:
  for i in range(len(row)):
    if abs(row[i]) > 1e5:
      row[i] = float("nan")
#  if row[0] > 4485:
#    break
  t.append(row[0])
  sail.append(row[3])
  rudder.append(row[4] * 180. / np.pi)
  yaw.append(norm_theta(row[5]) * 180. / np.pi)
  heel.append(norm_theta(row[6]) * 180. / np.pi)
  x.append(row[7])
  y.append(row[8])
  continue
  vx.append(row[10])
  vy.append(row[9])
  heading.append(np.arctan2(vy[-1], vx[-1]) * 180. / np.pi)
  leeway.append(norm_theta((heading[-1] - yaw[-1]) * np.pi / 180.) * 180. / np.pi)
  alphaw.append(np.arctan2(-row[2], -row[1]) * 180. / np.pi)
  wind_speed.append(np.sqrt(row[1] ** 2 + row[2] ** 2))
  true_alphaw.append(norm_theta(np.arctan2(-row[12], -row[11]) - row[5])* 180. / np.pi)
  true_wind_speed.append(np.sqrt(row[11] ** 2 + row[12] ** 2))

plt.plot(x, y, label="Boat Path")
#plt.plot([-76.477516, -76.475533, -76.474373, -76.477615, -76.479126], [38.98278, 38.98209, 38.98365, 38.985771, 38.983952], '*-', label="waypoints")
if False:
  plt.quiver(x, y, vx, vy, np.hypot(vx, vy))
  plt.colorbar(label="Speed (m/s)")
  plt.title("Boat Position (Wind is blowing bottom-right-to-top-left on screen)--Arrows and colors represent velocities")
  plt.xlabel("X position (deg longitude)")
  plt.ylabel("Y position (deg latitude)")
plt.legend()
plt.show()
sys.exit()
plt.figure()
ax = plt.subplot(111)
ax.plot(t, x - x[0], label='x less bias')
ax.plot(t, y - y[0], label='y less bias')
ax2 = ax.twinx()
ax2.plot(t, vx, 'c*', label='vx')
ax2.plot(t, vy, 'r*', label='vy')
ax2.plot(t, wind_speed, label='Wind Speed (m/s)')
ax2.plot(t, true_wind_speed, label='True Wind Speed (m/s)')
ax.legend(loc='upper left')
ax2.legend(loc='upper right')
plt.figure()
axyaw = plt.subplot(111, sharex=ax)
axyaw.plot(np.matrix(t).T, np.matrix(yaw).T + 0, label='Heading')
#axyaw.plot(t, heading, label='Vel Heading')
axyaw.plot(t, alphaw, label='Apparent Wind Angle')
#axyaw.plot(t, true_alphaw, 'm', label='True Wind Angle')
axrudder = axyaw.twinx()
axrudder.plot(t, rudder, 'r', label='Rudder')
axrudder.plot(t, heel, 'c', label='Heel');
axrudder.plot(t, leeway, 'y', label='Leeway Angle')
axrudder.plot(t, np.hypot(vx, vy) * 10, 'r--', label='Boat Speed')
axrudder.set_ylim([-45, 45])
axyaw.legend(loc='upper left')
axrudder.legend(loc='upper right')
plt.title('Boat data while beam reaching and close hauled')
axyaw.set_ylabel('Heading and Apparent Wind (upwind = 0) (deg)')
axrudder.set_ylabel('Rudder, Heel, and Leeway (deg)\n Boat Speed (tenths of a meter / sec)')
axyaw.set_xlabel('Time (sec)')
plt.grid()
plt.show()
