#!/usr/bin/python3
import numpy as np
from matplotlib import pyplot as plt
from theory_control import Physics, Norm, DebugForces, Controller
import read_replay

starttime = 4000.0
endtime = 4500.0
simlen = 100.0

(t, ox, oy, ovx, ovy, ospeed, oyaw, oheel, opitch, oomega, oheading,
 oleeway, sail, rudder, oalphaw, pitchvar, owind_speed, true_alphaw,
 true_wind_speed, heading_cmd, rudder_mode, truewx, truewy) = read_replay.read(
     "sep11replay-100Hz.csv", False, starttime, endtime)

x = []
y = []
vx = []
vy = []
speed = []
yaw = []
omega = []
heel = []
leeway = []
thetaw = []
wind_speed = []
deltasopt = []
deltaropt = []

# Create list of segments to run simulation over
startsegs = [0]
for i in range(len(t)):
  if t[i] > t[startsegs[-1]] + simlen:
    startsegs.append(i)
startsegs.append(len(t))

x0 = [0.0, 0.0]
physics = Physics()
control = Controller(physics)
forces = DebugForces()
for i in range(len(startsegs)-1):
  starti = startsegs[i]
  endi = startsegs[i+1]
  print(starti)
  ts = t[starti:endi]
  winds = [truewx[starti:endi], truewy[starti:endi]]
  deltass = sail[starti:endi]
  deltars = rudder[starti:endi]

  v0 = [ovx[starti], ovy[starti]]
  yaw0 = oyaw[starti]
  omega0 = oomega[starti]
  heel0 = oheel[starti]

  control = lambda i, t, tw, vw, tc, vc: (deltass[i], deltars[i])
  xs, ys, vxs, vys, yaws, omegas, heels, thetacs, vcs, thetaws, vws, _, _ = \
      physics.RunBase(
      ts, winds, x0, v0, yaw0, omega0, heel0, control,
      flopsail=True, debugf=forces)

  for j in range(len(thetaws)):
    ds = deltass[j]
    ds = abs(ds) if thetaws[j] > 0 else -abs(ds)
    dsopt, dropt = control.MaxForceForTorque(
        thetaws[j], vws[j], thetacs[j], vcs[j], ds, deltars[j])
    deltasopt.append(dsopt)
    deltaropt.append(dropt)
  forces.UpdateZero()
  x += xs
  y += ys
  vx += vxs
  vy += vys
  speed += vcs
  yaw += yaws
  omega += omegas
  heel += heels
  leeway += thetacs
  thetaw += thetaws
  wind_speed += vws

  x0 = [xs[-1], ys[-1]]

plt.figure()
ax = plt.subplot(111)
ax.plot(t, vx, 'c*', label='vx')
ax.plot(t, vy, 'r*', label='vy')
ax.plot(t, speed, 'g*', label='speed')
ax.plot(t, wind_speed, label='Wind Speed (m/s)')
ax.plot(t, true_wind_speed, label='True Wind Speed (m/s)')
ax.set_ylim([-5, 5])
ax.legend(loc='upper left')

plt.figure()
axyh = plt.subplot(111, sharex=ax)
axyh.plot(t, yaw, label='Yaw')
axyh.plot(t, oyaw, 'b--', label='Original Yaw')
axyh.plot(t, heel, 'g', label='Heel')
axyh.plot(t, oheel, 'g--', label='Original Heel')
axyh.legend()

plt.figure()
axyaw = plt.subplot(111, sharex=ax)
axyaw.plot(np.matrix(t).T, np.matrix(yaw).T + 0, 'b', label='Heading')
axyaw.plot(t, oyaw, 'b--', label='Orig Yaw')
axyaw.plot(t, [0] * len(t), 'k*', label='0s')
axyaw.plot(t, thetaw, 'g', label='Apparent Wind Angle')
axyaw.plot(t, heading_cmd, 'b-.', label='Heading Cmd')
axyaw.plot(t, np.hypot(vx, vy), 'k', label='Boat Speed')
axyaw.plot(t, np.array(ospeed), 'k--', label='Orig Boat Speed')
axrudder = axyaw.twinx()
axrudder.plot(t, rudder, 'r', label='Rudder')
axrudder.plot(t, sail, 'm', label='Sail')
axrudder.plot(t, heel, 'c', label='Heel');
axrudder.plot(t, oheel, 'c--', label='Orig Heel');
axrudder.plot(t, leeway, 'y', label='Leeway Angle')
axyaw.set_ylim([-np.pi, np.pi])
axrudder.set_ylim([-0.8, 0.8])
axyaw.legend(loc='upper left')
axrudder.legend(loc='upper right')
plt.title('Boat data while beam reaching and close hauled')
axyaw.set_ylabel('Heading and Apparent Wind (upwind = 0) (rad)\nBoat speed (m/s)')
axrudder.set_ylabel('Rudder, Heel, and Leeway (rad)')
axyaw.set_xlabel('Time (sec)')
plt.grid()

plt.figure()
axwind = plt.subplot(111, sharex=ax)
axwind.plot(t, true_wind_speed, 'r', label="True Wind Speed (m/s)")
axwind.plot(t, wind_speed, 'b', label="Apparent Wind Speed (m/s)")
axwind.set_ylim([0, 6])
axwinddir = axwind.twinx();
axwinddir.plot(t, true_alphaw, 'c', label="True Wind Dir (rad)")
axwind.legend(loc='upper left')
axwinddir.legend(loc='upper right')

plt.figure()
axtau = plt.subplot(111, sharex=ax)
axtau.plot(t, forces.taunet, label="Net Torque")
axtau.plot(t, forces.taus, label="Sail Torque")
axtau.plot(t, forces.tauk, label="Keel Torque")
axtau.plot(t, forces.taur, label="Rudder Torque")
axtau.plot(t, forces.tauB, label="Damping Torque")
axtau.set_ylim([-20, 20])
axtau.legend()

Fslon, Fslat = forces.Fslonlat()
Fklon, Fklat = forces.Fklonlat()
Frlon, Frlat = forces.Frlonlat()
plt.figure()
axflon = plt.subplot(211, sharex=ax)
plt.title('Longitudinal Forces')
axflat = plt.subplot(212, sharex=ax)
plt.title('Lateral Forces')
axflon.plot(t, forces.Flon, label="Net Longitudinal")
axflon.plot(t, Fslon, label="Sail")
axflon.plot(t, Fklon, label="Keel")
axflon.plot(t, Frlon, label="Rudder")
axflon.plot(t, forces.FBlon, label="Damping")
axflon.set_ylim([-20, 20])
axflon.legend()
axflat.plot(t, forces.Flat, label="Net Lateral")
axflat.plot(t, Fslat, label="Sail")
axflat.plot(t, Fklat, label="Keel")
axflat.plot(t, Frlat, label="Rudder")
axflat.plot(t, forces.FBlat, label="Damping")
axflat.set_ylim([-20, 20])
axflat.legend()

plt.figure()
axopt = plt.subplot(111, sharex=ax)
plt.title("Controller values for deltas, deltar")
axopt.plot(t, deltasopt, label="Sail Opt")
axopt.plot(t, deltaropt, label="Rudder Opt")
axopt.set_ylim([-0.8, 0.8])
axopt.legend()
plt.grid()

plt.show()
