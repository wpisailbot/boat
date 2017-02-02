#!/usr/bin/python3
import transformations
import numpy
from matplotlib import pyplot as plt

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

data = numpy.genfromtxt("basic_sim_data.csv", delimiter=',')[1:]
x = []
y = []
vx = []
vy = []
t = []
yaw = []
heel = []
pitch = []
heading = []
sail = []
rudder = []
yaw_true = []
alphaw = []
alphasail = []
goalr = []
goalh = []
for row in data:
  for i in range(len(row)):
    if abs(row[i]) > 1e5:
      row[i] = float("nan")
  t.append(row[0])
  sail.append(row[1])
  rudder.append(row[2])
  x.append(row[3])
  y.append(row[4])
  vx.append(row[5])
  vy.append(row[6])
  heading.append(numpy.arctan2(vy[-1], vx[-1]));
  q = row[7:11]
  e = transformations.euler_from_quaternion(q, axes='rzxy')
  yaw.append(e[0])
  heel.append(e[1])
  pitch.append(e[2])

  rotmat = transformations.quaternion_matrix(q)
  yaw_true.append(numpy.arctan2(rotmat[1, 0], rotmat[0, 0]))
  windx = row[11]
  windy = row[12]
  alphaw.append(numpy.arctan2(vy[-1] - windy, vx[-1] - windx) - yaw_true[-1])
  alphasail.append(alphaw[-1] - sail[-1])
  goalr.append(numpy.clip(yaw_true[-1] - .1, -.4, .4))
  goalh.append(row[46])

plt.plot(x, y, 'o')
plt.title("Boat Position (Wind is blowing left-to-right on screen)")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.figure()
ax = plt.subplot(111)
ax.plot(t, x, label='x')
ax.plot(t, y, label='y')
ax2 = ax.twinx()
ax2.plot(t, vx, 'c*', label='vx')
ax2.plot(t, vy, 'r*', label='vy')
ax.legend(loc='upper left')
ax2.legend(loc='upper right')
plt.figure()
plt.subplot(111, sharex=ax)
plt.plot(t, yaw, label='Heading')
plt.plot(t, heel, label='Heel')
plt.plot(t, pitch, label='Pitch')
plt.legend()
plt.figure()
plt.subplot(111, sharex=ax)
plt.plot(t, sail, label="Sail")
plt.plot(t, rudder, label="Rudder")
plt.plot(t, yaw_true, label="yaw")
plt.plot(t, alphaw, label="Apparent Wind")
plt.plot(t, alphasail, label="Sail AoA")
plt.plot(t, goalh, label="Goal Heading")
plt.plot(t, heading, label="true heading")
plt.legend()
plot_vec(data, 13, "Sail Force", ax)
plot_vec(data, 16, "Rudder Force", ax)
plot_vec(data, 19, "Keel Force", ax)
plot_vec(data, 22, "Hull Force", ax)
plot_vec(data, 25, "Net Force", ax)
plot_vec(data, 28, "Sail Torque", ax)
plot_vec(data, 31, "Rudder Torque", ax)
plot_vec(data, 34, "Keel Torque", ax)
plot_vec(data, 37, "Hull Torque", ax)
plot_vec(data, 40, "Righting Torque", ax)
plot_vec(data, 43, "Net Torque", ax)
plt.show()
