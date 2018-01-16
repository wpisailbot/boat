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

def plot_vec(d, starti, name, ax, maxy=50):
  t = data[:, 0]
  x = data[:, starti+0]
  y = data[:, starti+1]
  z = data[:, starti+2]
  plt.figure()
  plt.subplot(111, sharex=ax)
  plt.plot(t, x, label=name+" x")
  plt.plot(t, y, label=name+" y")
  plt.plot(t, z, label=name+" z")
#  plt.ylim(-maxy, maxy)
  plt.legend()

data = np.genfromtxt("sep11logsim.csv", delimiter=',')[:, :]
x = []
y = []
vx = []
vy = []
speed = []
t = []
yaw = []
heel = []
pitch = []
heading = []
leeway = []
sail = []
rudder = []
alphaw = []
pitchvar = []
wind_speed = []
true_alphaw = []
true_wind_speed = []
heading_cmd = []
rudder_mode = []
orig_yaw = []
orig_heel = []
orig_speed = []
for row in data:
  if row[0] < 4000:
    continue
  for i in range(len(row)):
    if abs(row[i]) > 1e5:
      row[i] = float("nan")
#  if row[0] > 4485:
#    break
  t.append(row[0])
  sail.append(row[3] * 180. / np.pi)
  rudder.append(row[4] * 180. / np.pi)
  yaw.append(norm_theta(row[5]) * 180. / np.pi)
  orig_yaw.append(norm_theta(row[20]) * 180. / np.pi)
  heel.append(norm_theta(row[6]) * 180. / np.pi)
  orig_heel.append(norm_theta(row[21]) * 180. / np.pi)
  pitch.append(norm_theta(row[7]) * 180. / np.pi)
  pitchvarstart = max(-100, -len(pitch))
  pitchvar.append(np.std(pitch[pitchvarstart:]))
  x.append(row[8])
  y.append(row[9])
  vx.append(row[10])
  vy.append(row[11])
  speed.append(np.hypot(vx[-1], vy[-1]))
  orig_speed.append(np.hypot(row[25], row[26]))
  heading.append(np.arctan2(vy[-1], vx[-1]) * 180. / np.pi)
  leeway.append(norm_theta((heading[-1] - yaw[-1]) * np.pi / 180.) * 180. / np.pi)
  alphaw.append(np.arctan2(row[2], row[1]) * 180. / np.pi)
  wind_speed.append(np.sqrt(row[1] ** 2 + row[2] ** 2))
  true_alphaw.append(norm_theta(np.arctan2(row[13], row[12]))* 180. / np.pi)
  true_wind_speed.append(np.sqrt(row[12] ** 2 + row[13] ** 2))
  heading_cmd.append(row[15] * 180. / np.pi)
  rudder_mode.append(row[16] * 10)

plt.plot(x, y, label="Boat Path")
#plt.plot([-76.477516, -76.475533, -76.474373, -76.477615, -76.479126], [38.98278, 38.98209, 38.98365, 38.985771, 38.983952], '*-', label="waypoints")
if False:
  plt.quiver(x, y, vx, vy, np.hypot(vx, vy))
  plt.colorbar(label="Speed (m/s)")
  plt.title("Boat Position (Wind is blowing bottom-right-to-top-left on screen)--Arrows and colors represent velocities")
  plt.xlabel("X position (deg longitude)")
  plt.ylabel("Y position (deg latitude)")
plt.legend()

plt.figure()
ax = plt.subplot(111)
ax.plot(t, x - x[0], label='x less bias')
ax.plot(t, y - y[0], label='y less bias')
ax2 = ax.twinx()
ax2.plot(t, vx, 'c*', label='vx')
ax2.plot(t, vy, 'r*', label='vy')
ax2.plot(t, speed, 'g*', label='speed')
ax2.plot(t, wind_speed, label='Wind Speed (m/s)')
ax2.plot(t, true_wind_speed, label='True Wind Speed (m/s)')
ax.legend(loc='upper left')
ax2.legend(loc='upper right')

plt.figure()
axyh = plt.subplot(111, sharex=ax)
axyh.plot(t, yaw, label='Yaw')
axyh.plot(t, orig_yaw, 'b--', label='Original Yaw')
axyh.plot(t, heel, 'g', label='Heel')
axyh.plot(t, orig_heel, 'g--', label='Original Heel')
axyh.plot(t, pitch, label='Pitch')
axyh.plot(t, [n * 100 for n in pitchvar], label='Pitch Stddev * 100')
axyh.legend()

plt.figure()
axyaw = plt.subplot(111, sharex=ax)
axyaw.plot(np.matrix(t).T, np.matrix(yaw).T + 0, 'b', label='Heading')
axyaw.plot(t, orig_yaw, 'b--', label='Orig Yaw')
axyaw.plot(t, alphaw, 'g', label='Apparent Wind Angle')
axyaw.plot(t, heading_cmd, 'b-.', label='Heading Cmd')
axyaw.plot(t, rudder_mode, 'r*', label='Rudder Mode')
#axyaw.plot(t, true_alphaw, 'm', label='True Wind Angle')
axrudder = axyaw.twinx()
axrudder.plot(t, rudder, 'r', label='Rudder')
axrudder.plot(t, sail, 'm', label='Sail')
axrudder.plot(t, heel, 'c', label='Heel');
axrudder.plot(t, orig_heel, 'c--', label='Orig Heel');
axrudder.plot(t, leeway, 'y', label='Leeway Angle')
axrudder.plot(t, np.hypot(vx, vy) * 10, 'k', label='Boat Speed')
axrudder.plot(t, np.array(orig_speed) * 10, 'k--', label='Orig Boat Speed')
axrudder.set_ylim([-45, 45])
axyaw.legend(loc='upper left')
axrudder.legend(loc='upper right')
plt.title('Boat data while beam reaching and close hauled')
axyaw.set_ylabel('Heading and Apparent Wind (upwind = 0) (deg)')
axrudder.set_ylabel('Rudder, Heel, and Leeway (deg)\n Boat Speed (tenths of a meter / sec)')
axyaw.set_xlabel('Time (sec)')
plt.grid()

plt.figure()
axwind = plt.subplot(111, sharex=ax)
axwind.plot(t, true_wind_speed, 'r', label="True Wind Speed (m/s)")
axwind.plot(t, wind_speed, 'b', label="Apparent Wind Speed (m/s)")
axwinddir = axwind.twinx();
axwinddir.plot(t, true_alphaw, 'c', label="True Wind Dir (deg)")
axwind.legend(loc='upper left')
axwinddir.legend(loc='upper right')

plot_vec(data, 27, "Sail Force", ax)
plot_vec(data, 30, "Rudder Force", ax)
plot_vec(data, 33, "Keel Force", ax)
plot_vec(data, 36, "Hull Force", ax)
plot_vec(data, 39, "Net Force", ax)
plot_vec(data, 42, "Sail Torque", ax)
plot_vec(data, 45, "Rudder Torque", ax)
plot_vec(data, 48, "Keel Torque", ax)
plot_vec(data, 51, "Hull Torque", ax)
plot_vec(data, 54, "Righting Torque", ax)
plot_vec(data, 57, "Net Torque", ax)
ax.set_xlim([4000, 4500])

plt.show()
