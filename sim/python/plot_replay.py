#!/usr/bin/python3
import numpy as np
import sys
from scipy import optimize
import scipy.stats.stats
from matplotlib import pyplot as plt
import read_replay

def norm_theta(theta):
  while (theta > np.pi):
    theta -= 2 * np.pi
  while (theta < -np.pi):
    theta += 2 * np.pi
  return theta

def norm_deg(theta):
  while (theta > 180.):
    theta -= 360.
  while (theta < -180.):
    theta += 360.
  return theta

def grid_minimize(f, xmin, xmax, nx):
  if len(nx) == 0:
    return [], f([])
  else:
    xopt = xmin
    fopt = float("inf")
    for x in np.linspace(xmin[0], xmax[0], nx[0]):
      newf = lambda xprime : f([x] + xprime)
      xprime, fprime = grid_minimize(newf, xmin[1:], xmax[1:], nx[1:])
      if fprime < fopt:
        xopt = [x] + xprime
        fopt = fprime
    return xopt, fopt


# Various functions for predicting rudder position
def PredictRudderSig2(ruds, heels, speeds, x0, params=[.6, .75, .25, 10., 80.]):
  dt = 0.1
  guess_heading = [x0[0]]
  guess_omega = [x0[1]]
  boatJ = params[0]
  boatB = params[1]
  boatheelK = params[2]
  divrud = params[3]
  rudmult = params[4]
  # sig2 provides reasonable fidelity over the range x = [-3, 3] and then
  # starts to flatten out. Effectively ignores the sign of x.
  sig2 = lambda x : (0.5 - 1. / (1. + np.exp(x))) ** 2 * (-1 if x < 0 else 1)
  for i in range(0, len(ruds)):
    rud = -rudmult * sig2(ruds[i] / divrud) # sign is reversed due to conventions...
    rud = heels[i] * boatheelK + rud
    w = guess_omega[-1]
    alpha = (rud * speeds[i] - boatB * w) / boatJ
    guess_omega.append(w + dt * alpha)
    guess_heading.append(guess_heading[-1] + guess_omega[-1] * dt)
  return guess_heading, guess_omega

def PredictRudderSig22(ruds, heels, speeds, x0, params=[.6, .75, .01, 10., 1.]):
  dt = 0.1
  guess_heading = [x0[0]]
  guess_omega = [x0[1]]
  boatJ = params[0]
  boatB = params[1]
  boatheelK = params[2]
  divrud = params[3]
  rudoffset = params[4]
  # sig2 provides reasonable fidelity over the range x = [-3, 3] and then
  # starts to flatten out. Effectively ignores the sign of x.
  sig2 = lambda x : (0.5 - 1. / (1. + np.exp(x))) ** 2 * (-1 if x < 0 else 1)
  kspeed = 1.0
  filt_speed = 1.
  for i in range(0, len(ruds)):
    rud = -sig2((ruds[i] + rudoffset) / divrud) # sign is reversed due to conventions...
#    rud = heels[i] * boatheelK + rud
    rud = 200. * rud * abs(rud) # 200. just gets things in scale
    w = guess_omega[-1]
    filt_speed = filt_speed + kspeed * (speeds[i] - filt_speed)
    alpha = (rud * filt_speed + heels[i] * boatheelK - boatB * w) / boatJ
    guess_omega.append(w + dt * alpha)
    guess_heading.append(guess_heading[-1] + guess_omega[-1] * dt)
  return guess_heading, guess_omega

def PredictError(t, ruds, heels, speeds, yaws, startt, endt, period, reset_amt, params):
  next_reset = startt + period
  tseg = []
  rudseg = []
  heelseg = []
  speedseg = []
  yawseg = []
  debug_t = []
  debug_headings = []
  debug_omegas = []
  debug_errors = []
  err = 0
  lastguess = yaws[0]
  lastomega = 0
  for i in range(0, len(t)):
    time = t[i]
    if time < startt:
      lastguess = yaws[i]
      continue

    tseg.append(time)
    rudseg.append(ruds[i])
    heelseg.append(heels[i])
    speedseg.append(speeds[i])
    yawseg.append(yaws[i])

    if time > next_reset or time > endt:
      theta0 = norm_deg(lastguess + norm_deg(yawseg[0] - lastguess) * reset_amt)
      x0 = [theta0, lastomega]
      guess_heading, guess_omega = PredictRudderSig22(
          rudseg, heelseg, speedseg, x0, params)
      debug_headings += guess_heading[:-1]
      debug_omegas += guess_omega[:-1]
      debug_t += tseg

      # guess_heading also approximates one past where it should...
      errs = [norm_deg(y - h) ** 2 for y, h in zip(yawseg, guess_heading[:-1])]
      err += sum(errs)
      debug_errors += errs

      next_reset += period
      lastguess = guess_heading[-1]
      lastomega = guess_omega[-1]
      tseg = []
      rudseg = []
      heelseg = []
      speedseg = []
      yawseg = []
    if time > endt:
      break
  return err, debug_t, debug_headings, debug_omegas, debug_errors

# For trying to analyze winds
def AngleAvg(data):
  if len(data) == 0:
    return 0
  base = data[0]
  diffs = sum([norm_deg(n - base) for n in data]) / float(len(data))
  avg = norm_deg(base + diffs)
  return avg
def AngleStd(data):
  if len(data) == 0:
    return 0, 0
  avg = AngleAvg(data)
  return 0, avg
  total = 0
  for n in data:
    total += norm_deg(n - avg) ** 2
  return float(np.sqrt(total / len(data))), avg
def GetBeforeAfterStddevAngle(arr, nbefore, nafter):
  before = []
  after = []
  bavgs = []
  aavgs = []
  for i in range(0, len(arr)):
    starti = max(0, i - nbefore)
    endi = min(len(arr), i + nafter)
    bstd, bavg = AngleStd(arr[starti:i])
    astd, aavg = AngleStd(arr[i:endi])
    before.append(bstd)
    after.append(astd)
    bavgs.append(bavg)
    aavgs.append(aavg)
  return before, after, bavgs, aavgs

data = np.genfromtxt("endurance-first-3-laps.csv", delimiter=',')[144:, :]
(t, x, y, vx, vy, speed, yaw, heel, pitch, omega, heading,
 leeway, sail, rudder, alphaw, pitchvar, wind_speed, true_alphaw,
 true_wind_speed, heading_cmd, rudder_mode, _, _) = read_replay.read("sep11replay.csv", True, 1000, 6000)
#(t, x, y, vx, vy, speed, yaw, heel, pitch, omega, heading,
# leeway, sail, rudder, alphaw, pitchvar, wind_speed, true_alphaw,
# true_wind_speed, heading_cmd, rudder_mode, _, _) = read_replay.read("/tmp/basic_replay_data.csv", True, 2700, 6000)

period = 10
reset_amt = 0.3
startt = 4000
endt = 4500

def fopt(x):
  if not isinstance(x, list):
    x = x.tolist()
  val, _, _, _, _ = PredictError(
      t, rudder, heel, speed, yaw, startt, endt, period, reset_amt, x)
  return val
#optparams, opterr = grid_minimize(
#    fopt, [.1, .1, 0.003, 5., 200.], [1., 1., .05, 20., 1000.], [5, 5, 5, 4, 4])
x0 = [0.6196, 0.5061, .002869, 4.388, 0.] # PRedictRudderSig22
#x0 = [1., 1., .25, 5., 80.] # PredictRudderSig2
#optparams = optimize.fmin(fopt, x0)
#optparams = optparams.tolist()
out = optimize.minimize(
    fopt, x0=x0,
    method='L-BFGS-B',
    bounds=[(0.3, 5.), (0.1, 3.), (0., 1.), (2., 25.), (0.0, 0.0)])#,
#    options={"maxfun":10})
optparams = out.x
opterr = out.fun
print("Opt params: ", optparams)
opterr, _, _, _, _ = PredictError(
    t, rudder, heel, speed, yaw, startt, endt, period, reset_amt, optparams)
print("Opt errors: ", opterr)
_, guesst, guess_heading, guess_omega, guess_error = PredictError(
    t, rudder, heel, speed, yaw, 0, 9000,
    100, 1, optparams)
guess_heading = [norm_deg(n) for n in guess_heading]

plt.plot(x, y, label="Boat Path")
#plt.plot([-76.477516, -76.475533, -76.474373, -76.477615, -76.479126], [38.98278, 38.98209, 38.98365, 38.985771, 38.983952], '*-', label="waypoints")
plt.figure()
if True:
  plt.quiver(x, y, vx, vy, np.hypot(vx, vy), scale=1e5, scale_units='xy')
  plt.colorbar(label="Speed (decim/s)")
  plt.title("Boat Position--Arrows and colors represent velocities")
  plt.xlabel("X position (deg longitude)")
  plt.ylabel("Y position (deg latitude)")
plt.legend()

plt.figure()
ax = plt.subplot(111)
ax.plot(t, x - x[0], label='x less bias')
ax.plot(t, y - y[0], label='y less bias')
ax2 = ax.twinx()
ax2.plot(t, vx, 'c--', label='vx')
ax2.plot(t, vy, 'r--', label='vy')
ax2.plot(t, speed, 'g--', label='speed')
ax2.plot(t, wind_speed, label='Wind Speed (m/s)')
ax2.plot(t, true_wind_speed, label='True Wind Speed (m/s)')
ax.legend(loc='upper left')
ax2.legend(loc='upper right')

plt.figure()
axyh = plt.subplot(111, sharex=ax)
axyh.plot(t, yaw, label='Yaw')
axyh.plot(guesst, guess_heading, 'b--', label='Yaw Guess')
axyh.plot(t, heel, label='Heel')
axyh.plot(t, pitch, label='Pitch')
axyh.plot(t, [n * 100 for n in pitchvar], label='Pitch Stddev * 100')
axyh.legend()

plt.figure()
axyaw = plt.subplot(111, sharex=ax)
axyaw.plot(np.matrix(t).T, np.matrix(yaw).T + 0, label='Heading')
axyaw.plot(t, alphaw, label='Apparent Wind Angle')
axyaw.plot(t, heading_cmd, 'b--', label='Heading Cmd')
axyaw.plot(t, rudder_mode, '*', label='Rudder Mode')
axyaw.plot(guesst, guess_heading, 'm--', label='Guessed Heading')
axyaw.set_ylim([-180, 180])
axrudder = axyaw.twinx()
axrudder.plot(t, rudder, 'r', label='Rudder')
axrudder.plot(t, sail, 'm', label='Sail')
axrudder.plot(t, heel, 'c', label='Heel');
#axrudder.plot(t, leeway, 'y', label='Leeway Angle')
axrudder.plot(t, omega, 'y', label='Omega')
axrudder.plot(guesst, guess_omega, 'c--', label='Guessed Omega')
axrudder.plot(t, np.hypot(vx, vy) * 10, 'r--', label='Boat Speed')
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
axwind.set_ylabel("Wind Speed (m/s)")
axwind.plot(t, true_wind_speed, 'r', label="True Wind Speed (m/s)")
axwind.plot(t, wind_speed, 'b', label="Apparent Wind Speed (m/s)")
axwinddir = axwind.twinx();
axwinddir.plot(t, true_alphaw, 'c', label="True Wind Dir (deg)")
axwinddir.set_ylabel("Wind direction (deg)")
axwind.legend(loc='upper left')
axwinddir.legend(loc='upper right')
axwind.set_xlabel("Time since start (sec)")
axwind.set_title("Wind Data")

plt.figure()
axerr = plt.subplot(111, sharex=ax)
axerr.plot(guesst, guess_error, label="Error in heading guess")
axerrspeed = axerr.twinx()
axerrspeed.plot(t, true_wind_speed, 'r', label="True Wind Speed (m/s)")
axerr.legend(loc='upper left')
axerrspeed.legend(loc='upper right')
plt.xlabel("Time since start (sec)")
axerr.set_ylabel("Squared error of yaw model, degrees^2")
axerrspeed.set_ylabel("Wind speed (m/s)")
plt.title("Wind speed and model error")

plt.figure()
axguess = plt.subplot(111, sharex=ax)
axmodelrud = axguess.twinx()
axmodelrud.plot(t, rudder, 'r:', label='Rudder (Commanded)')
axmodelrud.plot(t, heel, 'c', label='Heel')
axguess.plot(t, yaw, label='Yaw')
axguess.plot(guesst, guess_heading, 'b--', label='Modeled Heading')
axguess.plot(t, np.hypot(vx, vy) * 100, 'y', label='Speed')
axguess.legend(loc="upper left")
axmodelrud.legend(loc="upper right")
axguess.set_ylabel("Heading (Degrees), Speed (cm / s)")
axmodelrud.set_ylabel("Rudder/Heel (Degrees)")
axguess.set_xlabel("Time (sec)")
plt.title("Modeled and Actual Yaw")


overallstd, overallavg = AngleStd(true_alphaw)
print("Angle Avg: ", overallavg)
print("Angle Std: ", overallstd)
print("Speed Avg: ", np.mean(true_wind_speed))
print("Speed Std: ", np.std(true_wind_speed))

Nsec = 1.
beforestd, afterstd, beforeavg, afteravg = GetBeforeAfterStddevAngle(true_alphaw, int(Nsec / 0.1), int(Nsec / 0.1))
beforeavgdiff = [norm_deg(n - a) for n, a in zip(true_alphaw, beforeavg)]
afteravgdiff = [norm_deg(n - a) for n, a in zip(true_alphaw, afteravg)]
corr, pval = scipy.stats.stats.pearsonr(beforeavgdiff, afteravgdiff)
print("Corr:")
print(corr)
print("P-Va;:")
print(pval)
plt.figure()
axstd = plt.subplot(111, sharex=ax)
axstd.plot(t, afteravgdiff, label="After")
axstd.plot(t, beforeavgdiff, label="Before")
axstd.plot(t, [0] * len(t), '--')
plt.legend()

plt.figure()
plt.scatter(beforeavgdiff[0:-1:10], afteravgdiff[0:-1:10])
# TODO(james): Figure out windowing...
plt.title("Correlation between deviation from before and after t (r=%f; N=%fsec)" % (corr, Nsec))
plt.xlabel("Different from Mean Before t (deg)")
plt.ylabel("Difference from Mean After t (deg)")

speeddiff = [0] + [true_wind_speed[i] - true_wind_speed[i-1] for i in range(1, len(true_wind_speed))]
dwspeedbase = np.linspace(-1.5, 1.5, num=100)
sig = np.std(speeddiff)

plt.figure()
plt.hist(true_wind_speed, bins=20)
plt.xlabel("Wind Speed (m/s)")
plt.ylabel("Sample Count")
plt.title("True Wind Speed Histogram")
plt.figure()
sig *= 3.0
count, bins, _ = plt.hist(speeddiff, bins=20, log=True)
plt.plot(dwspeedbase, np.max(count) / (sig * np.sqrt(2 * np.pi)) * np.exp(-dwspeedbase ** 2 / (2.0 * sig**2)))
plt.xlabel("Change in wind speed from previous iteration")
plt.ylabel("Sample Count (Log Scale)")
plt.title("True Wind Speed difference Histogram")

#plt.figure()
#pitchfft = np.fft.fft(pitch)
#plt.plot(pitchfft)
#plt.title("Pitch FFT")
#
#plt.figure()
#windfft = np.fft.fft(true_wind_speed)
#plt.plot(windfft)
#plt.title("Wind Speed FFT")
#print("Avg error: ", np.mean(guess_error))
#print("Std dev: ", np.std(guess_error))
#print("Avg Sq Err: ", np.mean(np.square(guess_error)))

plt.show()
