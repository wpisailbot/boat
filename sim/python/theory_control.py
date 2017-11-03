#!/usr/bin/python3
import numpy as np
import sys
import scipy.signal
import scipy.stats.stats
from matplotlib import pyplot as plt
import unittest

def Norm(t):
  while t > np.pi:
    t -= 2 * np.pi
  while t < -np.pi:
    t += 2 * np.pi
  return t

def Sign(n):
  return 1.0 if n >= 0.0 else -1.0

class Airfoil(object):
  def __init__(self, A, rho, lifting=5.0, cmax=1.2):
    self.A = A # Cross-sectional area, m^2
    self.rho = rho # Density of medium, kg / m^2
    self.lifting = lifting
    self.Cmax = cmax

  def ClipAlpha(self, alpha):
    return np.clip(Norm(alpha), -np.pi / 2, np.pi / 2)

  def atanClCd(self, alpha):
    """
      Based on playing around with some common profiles,
      assuming a linear relationship to calculate
      atan2(Cl(alpha), Cd(alpha)) w.r.t. alpha
      seems reasonable.
    """
    clipalpha = self.ClipAlpha(alpha)
    deltaatan = -Sign(alpha) if abs(alpha) < np.pi / 2.0 else 0.0
    return (np.pi / 2.0 - abs(clipalpha)) * np.sign(clipalpha), deltaatan

  def normClCd(self, alpha):
    """
      Calculates sqrt(Cl^2 + Cd^2). This
      doesn't seem to capture typical profiles
      at particularly high angles of attack, but
      it seems a fair approximation. This may
      cause us to be more incliuned to sail
      straight downwind than we really should be.
      True profiles have a dip ~70-80 deg angle of attack.

      Returns norm, deltanorm/deltaalpha
    """
    alpha = self.ClipAlpha(alpha)
    exp = np.exp(-self.lifting * abs(alpha))
    norm = self.Cmax * (1.0 - exp)
    deltanorm = self.Cmax * self.lifting * exp * Sign(alpha)
    return norm, deltanorm

  def F(self, alpha, v):
    """
      Arguments:
        alpha: Airfoil angle of attack
        v: Relative speed in medium

      Returns:
        F, deltaF/deltaalpha: Note: deltaF does not account for heel
    """
    clipalpha = self.ClipAlpha(alpha)
    S = 0.5 * self.rho * self.A * v ** 2
    norm, deltanorm = self.normClCd(clipalpha)
    F = S * norm
    deltaF = S * deltanorm
    # Account for stupid angles of attack
    deltaF *= -1.0 if abs(alpha) > np.pi / 2.0 else 1.0
    return F, deltaF


class DebugForces(object):
  def __init__(self):
    self.taunet = []
    self.Flon = []
    self.Flat = []
    self.Fs = []
    self.Fk = []
    self.Fr = []
    self.gammas = []
    self.gammak = []
    self.gammar = []
    self.FBlon = []
    self.FBlat = []
    self.taus = []
    self.tauk = []
    self.taur = []
    self.tauB = []

  def UpdateZero(self):
    self.Update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0)

  def Update(self, taunet, Flon, Flat, Fs, Fk, Fr, gammas,
             gammak, gammar, FBlon, FBlat, taus, tauk, taur, tauB):
    self.taunet.append(taunet)
    self.Flon.append(Flon)
    self.Flat.append(Flat)
    self.Fs.append(Fs)
    self.Fk.append(Fk)
    self.Fr.append(Fr)
    self.gammas.append(gammas)
    self.gammak.append(gammak)
    self.gammar.append(gammar)
    self.FBlon.append(FBlon)
    self.FBlat.append(FBlat)
    self.taus.append(taus)
    self.tauk.append(tauk)
    self.taur.append(taur)
    self.tauB.append(tauB)

  def Flonlat(self, F, gamma):
    lon = [f * np.cos(g) for f, g in zip(F, gamma)]
    lat = [f * np.sin(g) for f, g in zip(F, gamma)]
    return lon, lat

  def Fslonlat(self):
    return self.Flonlat(self.Fs, self.gammas)
  def Fklonlat(self):
    return self.Flonlat(self.Fk, self.gammak)
  def Frlonlat(self):
    return self.Flonlat(self.Fr, self.gammar)

class Physics(object):
  def __init__(self):
    self.hs = 1.5 # Height of sail CoE above origin, m
    self.hk = -0.7 # Height of keel CoE above origin, m
    self.hr = 0.0 # Height of rudder CoE above origin, m

    # Positions longitudinally on the boat relative
    # to the origin, in m:
    self.rs = 0.1
    self.rk = 0.0
    self.rr = -0.9

    # Distance of the CoE from the rotational point (i.e.,
    # 0 would be a rudder that required no force to turn)
    self.ls = 0.25
    self.lr = 0.0

    rhowater = 1000.0 # Density of water, kg / m^3
    rhoair = 1.225 # Density of air, kg / m^3

    As = 2.0 # Sail Area, m^2
    Ak = .3 # Keel Area, m^2
    Ar = .04 # Rudder Area, m^2

    self.sail = Airfoil(As, rhoair, 5.0, 1.4)
    self.keel = Airfoil(Ak, rhowater, 8.0, 1.4)
    self.rudder = Airfoil(Ar, rhowater, 4.0, 1.7)

    self.Blon = 15.0 # Damping term, N / (m / s)
    self.Blat = 25.0 # Lateral damping term, bigger b/c hull long/thin)
#    self.Bv = 10.0
    self.Bomega = 500 # Damping term, N * m / (rad / sec)

    self.hb = -1.0 # Height of CoM of boat ballast, m
    self.wb = 14.0 * 9.8 # Weight of boat ballast, N

    self.J = 10.0 # Boat Moment of Inertia about yaw, kg * m^2
    self.m = 25.0 # Boat mass, kg

  def SailForces(self, thetaw, vw, deltas):
    """
      Calculates and returns forces from the sail.
      Arguments:
        thetaw: Wind, 0 = running downwind, +pi / 2 = wind from port
        vw: Wind speed, m / s
        deltas: Sail angle, 0 = all in, +pi / 2 = sail on starboard
        heel: Boat heel, 0 = upright
      Returns:
        Fs: Magnitude of force from sail (N)
        gammas: Angle of force from sail (rad, 0 = forwards, +pi / 2 = pushing to port)
        deltaFs: Derivative of Fs w.r.t. deltas
        deltagamma: Derivative of gamma w.r.t. deltas
    """
    alphas = -Norm(thetaw + deltas + np.pi)
    atanC, deltaatan = self.sail.atanClCd(alphas)
    Fs, deltaFs = self.sail.F(alphas, vw)
    #Fs = Fs if abs(alphas) > 0.08 else 0.0
    gammas = Norm(atanC - thetaw)
    deltaFs = deltaFs * -1.0 # -1 = dalpha / ddeltas
    deltagamma = deltaatan * -1.0 # -1 = dalpha / ddeltas
    return Fs, gammas, deltaFs, deltagamma

  def KeelForces(self, thetac, vc):
    """
      Calculates and returns forces from the sail.
      Arguments:
        thetac: Current, 0 = Boat going straight, +pi / 2 = Boat drifting to starboard
        vc: Speed in water, m / s
        heel: Boat heel, 0 = upright
      Returns:
        Fk: Magnitude of force from keel (N)
        gammak: Angle of force from keel (rad, 0 = forwards, +pi / 2 = pushing to port)
    """
    alphak = -Norm(thetac)
    atanC, _ = self.keel.atanClCd(alphak)
    atanC = (np.pi / 2.0 - 0.05) * np.sign(alphak)
    Fk, deltaFk = self.keel.F(alphak, vc)
    gammak = Norm(atanC - thetac + np.pi)
    return Fk, gammak

  def RudderForces(self, thetac, vc, deltar):
    """
      Calculates and returns forces from the sail.
      Arguments:
        thetac: Current, 0 = Boat going straight, +pi / 2 = Boat drifting to starboard
        vc: Speed in water, m / s
        deltar: Rudder angle, 0 = straight, + pi / 2 = rudder on starboard
        heel: Boat heel, 0 = upright
      Returns:
        Fr: Magnitude of force from rudder (N)
        gammar: Angle of force from rudder (rad, 0 = forwards, +pi / 2 = pushing to port)
        deltaFr: dFr / ddeltar
        deltagamma: dgammar / ddeltar
    """
    alphar = -Norm(thetac + deltar)
    alphar = np.clip(alphar, -.25, .25)
    atanC = (np.pi / 2.0 - 0.05) * Sign(alphar)
    Fr = 0.5 * self.rudder.A * self.rudder.rho * vc ** 2 * 5.0 * abs(alphar)
    gammar = Norm(atanC - thetac + np.pi)

    deltaFr = 0.5 * self.rudder.A * self.rudder.rho * vc ** 2 * 5.0 * -Sign(alphar)
    deltagamma = 0.0
    return Fr, gammar, deltaFr, deltagamma

  def SailTorque(self, Fs, gammas, deltas, heel, deltaFs,
      deltagammas, deltaheel):
    """
      Calculate yaw torque from sail, using output from SailForces
      Returns the torque and the derivative of the torque
      w.r.t. deltas.
    """
    sheel = np.sin(heel)
    cheel = np.cos(heel)
    cdeltas = np.cos(deltas)
    sdeltas = np.sin(deltas)
    return Fs * ((self.rs - self.ls * cdeltas) * np.sin(gammas) * cheel
                 + self.hk * np.cos(gammas) * sheel), 0.0
    r = np.sqrt((self.rs - self.ls * cdeltas) ** 2 + (self.hs * sheel) ** 2)
    drds = ((self.rs - self.ls * cdeltas) * (self.ls * sdeltas) \
        + (self.hs * sheel) * (self.hs * cheel) * deltaheel) \
        / r
    atany = -self.hs * sheel
    atanx = self.rs - self.ls * cdeltas
    theta = gammas - np.arctan2(atany, atanx)
    stheta = np.sin(theta)
    dsthetads = np.cos(theta) * \
        (deltagammas -
            (atanx * (-self.hs * cheel * deltaheel) -
             atany * (self.ls * cdeltas))
            / (atanx ** 2 + atany ** 2))
    dcheelds = -sheel * deltaheel
    tau = r * Fs * stheta * cheel
    dtauds = r * Fs * stheta * dcheelds \
             + r * Fs * dsthetads * cheel \
             + r * deltaFs * stheta * cheel \
             + drds * Fs * stheta * cheel
    return tau, dtauds

  def KeelTorque(self, Fk, gammak, heel):
    """
      Calculate yaw torque from keel, using output from KeelForces
    """
    return Fk * (self.rk * np.sin(gammak) * np.cos(heel)
                 + self.hk * np.cos(gammak) * np.sin(heel))
    r = np.sqrt(self.rk ** 2 + (self.hk * np.sin(heel)) ** 2)
    theta = gammak - np.arctan2(-self.hk * np.sin(heel), self.rk)
    return r * Fk * np.sin(theta) * np.cos(heel)

  def RudderTorque(self, Fr, gammar, heel, deltaFr, deltaheel):
    """
      Calculate yaw torque from rudder, using output from RudderForces
      Assumes self.hr is negligible.
    """
    tau = self.rr * Fr * np.sin(gammar) * np.cos(heel)
    dtaudr = self.rr * np.cos(heel) * deltaFr * np.sin(gammar)
    dtauds = -self.rr * Fr * np.sin(gammar) * np.sin(heel) * deltaheel
    dtauds = 0.0 # Not sure if above dtauds is still good.
    return tau, dtaudr, dtauds

  def ApproxHeel(self, Fs, gammas, Fk, gammak, deltaFs, deltagammas):
    """
      Returns equilibrium heel angle for a given Fs, Fk,
      as well as the derivative of the heel with respect
      to deltas
    """
    tanheel = (Fs * self.hs * np.sin(gammas) + Fk * self.hk * np.sin(gammak)) / (self.hb * self.wb)
    heel = np.arctan(tanheel)
    dheel = self.hs * (deltaFs * np.sin(gammas) + Fs * np.cos(gammas) * deltagammas) \
        / ((1.0 + tanheel ** 2) * self.hb * self.wb)
    return heel, dheel

  def NetForce(self, thetaw, vw, thetac, vc, deltas, deltar, heel, omega, debugf=None):
    """
      Sum up all the forces and return net longitudinal and lateral forces, and net torque
      Arguments:
        thetaw: Wind dir
        vw: wind speed
        thetac: Water dir
        vc: Water speed
        deltas: sail angle
        deltar: rudder angle
        heel: Duh.
        omega: boat rotational velocity, rad / s
        debugf: DebugForces instance for... debugging
      Returns: Flon, Flat, taunet, newheel
    """
    Fs, gammas, dFsds, dgsds= self.SailForces(thetaw, vw, deltas)
    Fk, gammak = self.KeelForces(thetac, vc)
    heel, dheelds = self.ApproxHeel(Fs, gammas, Fk, gammak, dFsds, dgsds)

    Fr, gammar, dFrdr, dgrdr = self.RudderForces(thetac, vc, deltar)
    taus, dtausds = self.SailTorque(Fs, gammas, deltas, heel, dFsds, dgsds, dheelds)
    tauk = self.KeelTorque(Fk, gammak, heel)
    taur, dtaurdr, dtaurds = self.RudderTorque(Fr, gammar, heel, dFrdr, dheelds)
    tauB = -self.Bomega * omega * abs(omega)
    FBlon = -self.Blon * vc * abs(vc) * np.cos(thetac)
    FBlat = self.Blat * vc * np.sin(thetac)
    Flon = Fs * np.cos(gammas) + Fk * np.cos(gammak) + Fr * np.cos(gammar) + FBlon
    Flat = (Fs * np.sin(gammas) + Fk * np.sin(gammak) + Fr * np.sin(gammar)) * np.cos(heel) + FBlat
    taunet = taus + tauk + taur + tauB
    newheel, _ = self.ApproxHeel(Fs, gammas, Fk, gammak, 0, 0)

    #print("Flon: ", Flon, " Flat: ", Flat, " Blon: ", -self.Blon * vc * np.cos(thetac),
    #    " Fs ", Fs, " gammas ", gammas, " Fk ", Fk, " gammak ", gammak, " Fr ", Fr,
    #    " gammar ", gammar)
    #print("taunet ", taunet, " taus ", taus, " tauk ", tauk, " taur ", taur, " Btau",
    #    -self.Bomega * omega)
    if debugf != None:
      debugf.Update(taunet, Flon, Flat, Fs, Fk, Fr, gammas,
          gammak, gammar, FBlon, FBlat, taus, tauk, taur, tauB)
    return Flon, Flat, taunet, newheel

  def Yadaptive(self, thetaw, vw, thetac, vc, yaw, omega, deltas, deltar):
    """
      Using: u = {F_lon, tau_net}
          beta = {Blon, Bomega, Ar, rs, taubias, 1}
    """

    YFlonBlon = -vc * abs(vc) * np.cos(thetac)
    Fr, gammar, _, _ = self.RudderForces(thetac, vc, deltar)
    YFlonAr = Fr * np.cos(gammar) / self.rudder.A
    Fs, gammas, _, _= self.SailForces(thetaw, vw, deltas)
    Fk, gammak = self.KeelForces(thetac, vc)
    YFlonconst = Fs * np.cos(gammas) + Fk * np.cos(gammak)
    YFlon = np.matrix([[YFlonBlon, 0.0, YFlonAr, 0.0, 0.0, YFlonconst]])

    heel, _ = self.ApproxHeel(Fs, gammas, Fk, gammak, 0.0, 0.0)
    taur, _, _ = self.RudderTorque(Fr, gammar, heel, 0.0, 0.0)
    tauk = self.KeelTorque(Fk, gammak, heel)
    taus, _ = self.SailTorque(Fs, gammas, deltas, heel, 0.0, 0.0, 0.0)
    YtauBomega = -omega * abs(omega)
    YtauAr = taur / self.rudder.A
    Ytaurs = Fs * np.sin(gammas) * np.cos(heel)
    Ytauconst = tauk + (taus - Ytaurs * self.rs)
    Ytau = np.matrix([[0.0, YtauBomega, YtauAr, Ytaurs, 1.0, Ytauconst]])
    print("Ytau: ", Ytau)
    print("YFlon: ", YFlon)

    return np.concatenate((YFlon, Ytau), axis=0)

  def Update(self, truewx, truewy, x, y, vx, vy, yaw, omega, deltas, deltar,
             heel, dt, flopsail=False, debugf=None):
    thetac = -Norm(np.arctan2(vy, vx) - yaw)
    vc = np.sqrt(vx ** 2 + vy ** 2)

    appwx = truewx - vx
    appwy = truewy - vy
    thetaw = Norm(-np.arctan2(appwy, appwx) + yaw)
    vw = np.sqrt(appwx ** 2 + appwy ** 2) * 1.6 # For wind gradient

    if flopsail:
      deltas = abs(deltas) if thetaw > 0 else -abs(deltas)

    #print("thetac ", thetac, " vc ", vc, " thetaw ", thetaw, " vw ", vw)

    Flon, Flat, tau, newheel = self.NetForce(
        thetaw, vw, thetac, vc, deltas, deltar, heel, omega, debugf)

    if False:
      # For approximating damping force, with overall force as input,
      # state as [pos, vel]
      Ac = np.matrix([[0.0, 1.0],
                      [0.0, -self.Bv / self.m]])
      Bc = np.matrix([[0.0], [1.0 / self.m]])
      (Ad, Bd, _, _, _) = scipy.signal.cont2discrete((Ac, Bc, Ac, Bc), dt)

      statex = np.matrix([[x], [vx]])
      forcex = Flon * np.cos(yaw) - Flat * np.sin(yaw)
      statex = Ad * statex + Bd * forcex

      statey = np.matrix([[y], [vy]])
      forcey = Flon * np.sin(yaw) + Flat * np.cos(yaw)
      statey = Ad * statey + Bd * forcey

      x = statex[0, 0]
      y = statey[0, 0]
      vx = statex[1, 0]
      vy = statey[1, 0]
    else:
      ax = (Flon * np.cos(yaw) - Flat * np.sin(yaw)) / self.m
      ay = (Flon * np.sin(yaw) + Flat * np.cos(yaw)) / self.m
      x += vx * dt + 0.5 * ax * dt ** 2
      y += vy * dt + 0.5 * ay * dt ** 2
      vx += ax * dt
      vy += ay * dt

    alpha = tau / self.J
    yaw += omega * dt + 0.5 * alpha * dt ** 2
    yaw = Norm(yaw)
    omega += alpha * dt

    kHeel = 0.3
    heel = heel + (1.0 - np.exp(-kHeel * dt)) * (newheel - heel)
#    heel = newheel

    thetac = -Norm(np.arctan2(vy, vx) - yaw)
    vc = np.sqrt(vx ** 2 + vy ** 2)

    return x, y, vx, vy, yaw, omega, heel, thetac, vc, thetaw, vw

  def RunBase(self, ts, winds, x0, v0, yaw0, omega0, heel0, control,
              flopsail=False, debugf=None):
    """
      ts: Times to simulate over, e.g. [0, .1, .2, .3, .4]
         to simulate 4 steps of 0.1sec each
      winds: list of two lists, where each sublist
         is of length ts and contains the true wind
         at that time
      x0: list of length 2 = (x, y) initial positoin
      v0: list of length 2 = (x, y) initial velocity
      yaw0: float, initial yaw
      omega0: float, initial time derivative of yaw
      heel0: float, intitial heel
      control: Function, of the form:
        Params:
          i: current index from ts/winds that we are at
          t: ts[i]
          thetaw: Apparent wind dir
          vw: Apparent wind vel
          thetac: Apparent current
          vc: Apparent water speed
        Returns: deltas, deltar
    """
    xs = [x0[0]]
    ys = [x0[1]]
    vxs = [v0[0]]
    vys = [v0[1]]
    yaws = [yaw0]
    omegas = [omega0]
    heels = [heel0]
    vcs = [np.hypot(v0[0], v0[1])]
    thetacs = [Norm(np.arctan2(v0[1], v0[0]) + yaws[0])]
    vws = [0.0]
    thetaws = [0.0]
    deltass = []
    deltars = []
    for i in range(1, len(ts)):
      dt = np.clip(ts[i] - ts[i - 1], 0.001, 0.2)
      wx = winds[0][i]
      wy = winds[1][i]
      deltas, deltar = control(
          i, ts[i], thetaws[-1], vws[-1], thetacs[-1], vcs[-1], yaws[-1], omegas[-1])
      deltass.append(deltas)
      deltars.append(deltar)
      x, y, vx, vy, yaw, omega, heel, thetac, vc, thetaw, vw = self.Update(
          wx, wy, xs[-1], ys[-1], vxs[-1], vys[-1], yaws[-1], omegas[-1],
          deltas, deltar, heels[-1], dt, flopsail, debugf)

      if abs(vx) > 100:
        vx = 0
        vy = 0
        omega = 0
        heel = 0

      xs.append(x)
      ys.append(y)
      vxs.append(vx)
      vys.append(vy)
      yaws.append(yaw)
      omegas.append(omega)
      heels.append(heel)
      thetacs.append(thetac)
      vcs.append(vc)
      thetaws.append(thetaw)
      vws.append(vw)

    deltass.append(0.0)
    deltars.append(0.0)

    return xs, ys, vxs, vys, yaws, omegas, heels, thetacs, vcs,\
        thetaws, vws, deltass, deltars

  def Run(self, wind, v0, omega0, heel0, control, dt=0.01, niter=200, flopsail=True, debugf=None):
    wx = [wind[0]] * niter
    wy = [wind[1]] * niter
    winds = [wx, wy]
    ts = [i * dt for i in range(niter)]
    return self.RunBase(ts, winds, [0.0, 0.0], v0, 0.0, omega0, heel0,
                        control, flopsail=flopsail, debugf=debugf)
    xs = [0]
    ys = [0]
    vxs = [v0[0]]
    vys = [v0[1]]
    yaws = [0]
    omegas = [omega0]
    heels = [heel0]
    vcs = [np.hypot(v0[0], v0[1])]
    thetacs = [Norm(np.arctan2(v0[1], v0[0]) + yaws[0])]

    for i in range(niter):
      print(i * dt)
      x, y, vx, vy, yaw, omega, heel, thetac, vc = self.Update(
          wx, wy, xs[-1], ys[-1], vxs[-1], vys[-1], yaws[-1], omegas[-1],
          deltas, deltar, heels[-1], dt)
      if abs(vx) > 100:
        vx = 0
        vy = 0
        omega = 0
        heel = 0

      xs.append(x)
      ys.append(y)
      vxs.append(vx)
      vys.append(vy)
      yaws.append(yaw)
      omegas.append(omega)
      heels.append(heel)
      thetacs.append(thetac)
      vcs.append(vc)

    return xs, ys, vxs, vys, yaws, omegas, heels, thetacs, vcs

class Controller(object):
  def __init__(self, physics):
    self.physics = physics
    self.maxrud = 0.25

    self.Qtau = 0.01
    self.Qf = 1.0

    self.Kbeta = 0.0 * np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    self.beta = np.matrix([[physics.Blon],
                           [physics.Bomega],
                           [physics.rudder.A],
                           [physics.rs],
                           [0.0],
                           [1.0]])
    self.betamin = np.matrix([[0.0],
                              [0.0],
                              [0.01],
                              [-1.0],
                              [-10.0],
                              [1.0]])
    self.betamax = np.matrix([[1000.0],
                              [10000.0],
                              [1.0],
                              [1.0],
                              [10.0],
                              [1.0]])
    self.Lambda = np.diag([1.0, 1.0])
    self.lastt = float("nan")
    self.betas = []

  def ClipSail(self, deltas, thetaw):
    maxsail = abs(Norm(np.pi - thetaw))
    return np.clip(deltas, 0.0 if thetaw > 0.0 else -maxsail,
                           maxsail if thetaw > 0.0 else 0.0)

  def ClipRudder(self, deltar, thetac):
    return np.clip(deltar, -self.maxrud - thetac, self.maxrud - thetac)

  def Adapt(self, thetaw, vw, thetac, vc, yaw, omega, deltas, deltar,
            goalyaw, goalomega):
    # u = Y beta
    # u = u_r + diff = Y beta
    Y = self.physics.Yadaptive(
        thetaw, vw, thetac, vc, yaw, omega, deltas, deltar)
    yawdiff = Norm(goalyaw - yaw)
    omegadiff = goalomega - omega
    vcgoal = vc
    diff = np.matrix([[0.0], [omegadiff]]) +\
           self.Lambda * np.matrix([[vcgoal - vc], [yawdiff]])
    print("diff: ", diff)
    print("dot: ", (Y.T * diff).T)
    betadot = -self.Kbeta * Y.T * diff
    return betadot

  def ControlMaxForce(self, i, t, thetaw, vw, thetac, vc, yaw, omega):
    dt = t - self.lastt
    if np.isnan(self.lastt):
      dt = 0.0
    self.lastt = t
#    self.Qtau = 1.0
    goalyaw = -np.pi / 2.0
    goalomega = 0.0
    taue = 10.0 * Norm(goalyaw - yaw) + (goalomega - omega) * 0.0\
           - self.beta[4, 0]
    #taue = 0.0
    constraint = 0.0
    _, _, _, _, _, deltas, deltar = control.GlobalMaxForceTorque(
        thetaw, vw, thetac, vc, taue, constraint, 20)

    if np.isnan(deltas) and constraint == 0.0:
      return 0.0, 0.0

    betadot = self.Adapt(
        thetaw, vw, thetac, vc, yaw, omega, deltas, deltar,
        goalyaw, goalomega)

    self.beta += betadot * dt
    self.beta = np.clip(self.beta, self.betamin, self.betamax)
    self.betas.append(self.beta)
    print(self.beta.T)
    self.physics.rudder.A = self.beta[2, 0]
    self.physics.rs = self.beta[3, 0]

    if np.isnan(deltas) and constraint != 0:
      taue = -np.sign(constraint) * float("inf")
      _, _, _, _, _, deltas, deltar = control.GlobalMaxForceTorque(
          thetaw, vw, thetac, vc, taue, constraint, 20)
    return deltas, deltar

  def ControlGradDescent(self, i, t, thetaw, vw, thetac, vc, omega):
    ds = np.clip(Norm(np.pi - thetaw), -np.pi / 2.0, np.pi / 2.0)
    print("thetaw ", thetaw, " ds ", ds)
    deltari = 0.25
    return self.MaxForceForTorque(thetaw, vw, thetac, vc, ds, deltari)

  def TorqueConstrainedRudder(
      self, taus, tauk, taug, heel, thetac, vc):
    CR = 5.0 # TODO: parameterize properly.
    denom = 0.5 * self.physics.rudder.rho * self.physics.rudder.A\
        * vc ** 2 * CR * np.cos(thetac) * np.cos(heel) * self.physics.rr
    return -(taus + tauk - taug) / denom - thetac

  def GlobalMaxForceTorque(
    self, thetaw, vw, thetac, vc, taug, constraint, nsteps):
    """
      Parameters:
        thetaw, vw, thetac, vc: Wind and current directions and speeds
        taug: Nominal torque to work to
        constraint: Whether we should attempt equality (0.0),
          maximize (1.0), or minimize (-1.0) torque, using taug
          as a constraint either for the equality or as a lower/upper
          bound on the torque.
        nsteps: Number of sail positions to consider.
    """
    maxsail = abs(Norm(np.pi - thetaw))
    minsail = maxsail - np.pi / 2.0
    minds = max(0.0, minsail) if thetaw > 0.0 else -maxsail
    maxds = maxsail if thetaw > 0.0 else min(-minsail, 0.0)
    mindr = -self.maxrud - thetac
    maxdr = self.maxrud - thetac

    deltass = []
    deltars = []
    Flons = []
    taues = []
    costs = []

    mincost = float("inf")
    deltasmax = float("nan")
    deltarmax = float("nan")

    hardtorque = constraint == 0.0
    maximizetau = constraint > 0.0 # Worry about Q_F?

    for deltas in np.linspace(minds, maxds, num=nsteps):
      Fs, gammas, dFsds, dgsds = self.physics.SailForces(
          thetaw, vw, deltas)
      Fk, gammak = self.physics.KeelForces(thetac, vc)
      heel, dheelds = self.physics.ApproxHeel(
          Fs, gammas, Fk, gammak, dFsds, dgsds)
      taus, dtausds = self.physics.SailTorque(
          Fs, gammas, deltas, heel, dFsds, dgsds, dheelds)
      tauk = self.physics.KeelTorque(Fk, gammak, heel)

      deltarconstrained = self.TorqueConstrainedRudder(
          taus, tauk, taug, heel, thetac, vc)

      deltar = deltarconstrained
      if hardtorque:
        deltar = self.ClipRudder(deltar, thetac)
      else:
        # Increasing deltar decreases torque (normally)
        if maximizetau:
          deltar = min(deltarconstrained, mindr)
        else:
          deltar = max(deltarconstrained, maxdr)

      if self.ClipRudder(deltar, thetac) != deltar:
        # Invalid result for rudder.
        continue

      Fr, gammar, dFrdr, dgrdr = self.physics.RudderForces(
          thetac, vc, deltar)
      taur, dtaurdr, dtaurds = self.physics.RudderTorque(
          Fr, gammar, heel, dFrdr, dheelds)

      Flon = Fs * np.cos(gammas) + Fk * np.cos(gammak) \
             + Fr * np.cos(gammar)

      taue = taus + tauk + taur
      # Could be signtau = -np.sign(constraint)
      signtau = 0.0 if hardtorque else (-1.0 if maximizetau else 1.0)
      cost = signtau * self.Qtau * taue - self.Qf * Flon
      if hardtorque:
        cost += self.Qtau * (taue - taug) * (taue - taug)

      deltass.append(deltas)
      deltars.append(deltar)
      Flons.append(Flon)
      taues.append(taue)
      costs.append(cost)

      if cost < mincost:
        mincost = cost
        deltasmax = deltas
        deltarmax = deltar

    return deltass, deltars, Flons, taues, mincost, deltasmax, deltarmax

  def MaxForceForTorque(self, thetaw, vw, thetac, vc, deltasi, deltari):
    """
      Given a particular set of conditions, adjusts deltar
      and deltas to optimize for net forwards force while
      maintaining the torque generated by the original
      conditions.
    """
    laststep = 0.0
    deltasstep = 0.0
    taunom = float('nan')
    clipr = deltari
    clips = deltasi
    deltar = deltari
    deltas = deltasi
    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    print("thetaw ", thetaw, " vw ", vw, " thetac ", thetac, " vc ", vc, " deltasi ", deltasi, " deltari ", deltari)
    while deltasstep * laststep >= 0.0:# or np.isnan(taunom):
      print("Iter")
      Fs, gammas, dFsds, dgsds = self.physics.SailForces(thetaw, vw, deltas)
#      print("Fs ", Fs, " gammas ", gammas, " dFsds ", dFsds, " dgsds ", dgsds)
      Fk, gammak = self.physics.KeelForces(thetac, vc)
      heel, dheelds = self.physics.ApproxHeel(Fs, gammas, Fk, gammak, dFsds, dgsds)
      Fr, gammar, dFrdr, dgrdr = self.physics.RudderForces(thetac, vc, deltar)
      taus, dtausds = self.physics.SailTorque(Fs, gammas, deltas, heel, dFsds, dgsds, dheelds)
      # Ignore the keel...
#      print("Fr ", Fr, " gammar ", gammar, " dFrdr ", dFrdr, " dgrdr", dgrdr)
      taur, dtaurdr, dtaurds = self.physics.RudderTorque(Fr, gammar, heel, dFrdr, dheelds)
      taunet = taus + taur
      if np.isnan(taunom):
        taunom = taunet
#        print("Taunom: ", taunom)
      tauerr = taunet - taunom
      print("tauerr: ", tauerr)

      dFlonds = dFsds * np.cos(gammas) - Fs * np.sin(gammas) * dgsds
#      print("dFlonds: ", dFlonds, " taunet: ", taunet)

      laststep = deltasstep
      deltasstep = 0.01 * Sign(dFlonds)
      deltas += deltasstep
      dtau = dtausds * deltasstep + dtaurds * deltasstep
#      print("dtau ", dtau, " dtausds ", dtausds, " dtaurds ", dtaurds, " dtaurdr ", dtaurdr)
      deltarstep = -(dtau + tauerr) / dtaurdr
      deltar += deltarstep

      clips = self.ClipSail(deltas, thetaw)
      clipr = self.ClipRudder(deltar, thetac)
      print("clips ", clips, " clipr ", clipr)
      if clips != deltas or clipr != deltar:
#        print("breaking due to limit")
        break

    return clips, clipr

# TODO:
# Only reduce longitudinal sail/rudder authority when heeled.
# For control strategy:
# Maximize forwards force while providing at least X turning torque,
# provide range of turning torques to planner, generate approximatino
# of max forwards force as functino of turning torque, supply leeways.
# Maximum allowable torque is the maximum generatable from the rudder
# with current heel and sail. From there, we then begin to try
# to improve forwards force by following the gradient (we adjust the
# sail and then adjust the rudder, iteratively).

def SailForcesAndTorque(physics, thetaw, vw, thetac, vc, deltas):
  Fs, gammas, _, _ = physics.SailForces(thetaw, vw, deltas)
  Fk, gammak = physics.KeelForces(thetac, vc)
  heel, _ = physics.ApproxHeel(Fs, gammas, Fk, gammak, 0.0, 0.0)
  taus, _ = physics.SailTorque(Fs, gammas, deltas, heel, 0, 0, 0)
  Fslon = Fs * np.cos(gammas)
  return Fslon, taus, heel

def PlotSail(physics, thetaw, vw, thetac, vc, fname=None):
  maxsail = abs(Norm(np.pi - thetaw))
  minsail = maxsail - np.pi / 2.0
  minds = max(0.0, minsail) if thetaw > 0.0 else -maxsail
  maxds = maxsail if thetaw > 0.0 else min(-minsail, 0.0)

  Fss = []
  tauss = []
  heels = []
  deltass = np.arange(minds, maxds, 0.01)
  for deltas in deltass:
    Fs, taus, heel = SailForcesAndTorque(
        physics, thetaw, vw, thetac, vc, deltas)
    Fss.append(Fs)
    tauss.append(taus)
    heels.append(heel)

  tack = "running" if abs(thetaw) < 0.5 else \
             "broad reach" if abs(thetaw) < 1.4 else \
                 "beam reach" if abs(thetaw) < 1.8 else \
                     "close reach" if abs(thetaw) < 2.5 else \
                         "close hauled" if abs(thetaw) < 2.8 else \
                             "in irons"

  plt.figure()
  plt.title("Sail Forces for Various $\delta_s$, thetaw=%f (%s)" % (thetaw, tack))
  plt.plot(deltass, Fss, label="Sail forward force ($F_{s,lon}$)")
  plt.plot(deltass, tauss, label="Sail yaw torque ($\\tau_s$)")
  plt.xlabel("Sail angle, $\delta_s$ (radians), from %s (left) to %s (right)"
      % ("fully stalled" if thetaw > 0 else "luffing",
         "fully stalled" if thetaw <= 0 else "luffing"))
  plt.ylabel("Force (N), Torque (N-m)")
  plt.legend(loc='upper left')
  ax = plt.twinx()
  ax.plot(deltass, heels, 'r', label="Heel angle ($\psi$)")
  ax.set_ylabel("Heel Angle (radians)")
  ax.legend(loc='upper right')

  plt.xlim((minds, maxds))
  if fname != None:
    plt.savefig(fname)

def PlotMaxForceForTorque(control, thetaw, vw, thetac, vc, taue, nsteps):
  deltass, deltars, Flons, taues, mincost, deltasmax, deltarmax = \
      control.GlobalMaxForceTorque(thetaw, vw, thetac, vc, taue, 0.0, nsteps)

  plt.figure()
  plt.plot(deltass, Flons, label="$F_{lon}$")
  plt.plot(deltass, taues, label="$\\tau_e$")
  plt.legend(loc='upper left')
  ax = plt.twinx()
  ax.plot(deltass, deltars, 'r', label="$\delta_r$")
  ax.legend(loc='upper right')

if __name__ == "__main__":
  sim = Physics()
  wind = [3.0, 0.0]
  v0 = [0.0, 0.0]
  omega0 = 0.0
  heel0 = 0.0
  deltas = 0.0
  deltar = 0.25
  dt = 0.01
  niter = 5000
  t = [dt * n for n in range(niter)]
  forces = DebugForces()
  control = lambda i, t, tw, vw, tc, vc, yaw, om: (deltas, deltar)
  xs, ys, vxs, vys, yaws, omegas, heels, thetacs, vcs, thetaws, vws, _, _ =\
      sim.Run(
      wind, v0, omega0, heel0, control, dt=dt, niter=niter)

  if 0:
    PlotSail(sim, 0.001, 3.0, 0.0, 1.0)
    PlotSail(sim, np.pi / 4.0, 3.0, 0.0, 1.0)
    PlotSail(sim, np.pi / 2.0, 3.0, 0.0, 1.0, 'sail_forces_beam.eps')
    PlotSail(sim, 3 * np.pi / 4.0, 3.0, 0.0, 1.0)
    PlotSail(sim, 7 * np.pi / 8.0, 3.0, 0.0, 1.0)
    PlotSail(sim, 3.0, 3.0, 0.0, 1.0)

  sim.rs -= 0.1
  sim.hs *= 1.5
  sim.Blon += 10
  sim.keel.A *= 1.2
#  sim.sail.A *= 0.8
  sim.rr *= 0.85
  sim.Blat *= 1.1
  sim.Bomega *= 0.2
  sim.J *= 1.5
  controlsim = Physics()
  control = Controller(controlsim)

  if 0:
    PlotMaxForceForTorque(control, np.pi / 2.0, 3.0, 0.05, 0.4, -2.0, 50)
#  control.MaxForceForTorque(-1.51716946346, 4.56503205727,
#      -0.0564452767422, 0.648521086573, -1.57079632679, 0.25)
#  control.MaxForceForTorque(-1.51638429183, 4.56599217829,
#      -0.0695781219434, 0.640581832306, -1.57079632679, 0.25)
#  control.MaxForceForTorque(-1.51716946346, 4.56503205727,
#      -0.0564452767422, 0.648521086573, -1.57079632679, 0.25)
#  control.MaxForceForTorque(-1.51638429183, 4.56599217829,
#      -0.0695781219434, 0.640581832306, -1.57079632679, 0.25)
#  sys.exit()
  #deltasopt = []
  #deltaropt = []
  #for i in range(len(thetaws)):
  #  print(i)
  #  ds = deltasopt[-1] if len(deltasopt) > 0 else deltas
  #  ds = np.clip(Norm(np.pi - thetaws[i]), -np.pi / 2.0, np.pi / 2.0)
  #  ds = abs(ds) if thetaws[i] > 0 else -abs(ds)
  #  dsopt, dropt = control.MaxForceForTorque(
  #      thetaws[i], vws[i], thetacs[i], vcs[i], ds, deltar)
  #  print("ds ", dsopt, " dr ", dropt)
  #  deltasopt.append(dsopt)
  #  deltaropt.append(dropt)

  plt.figure()
  axxy = plt.subplot(111)
  axxy.plot(t, xs, 'b', label="x")
  axxy.plot(t, ys, 'g', label="y")
  axxy.plot(t, vxs, 'b*', label="vx")
  axxy.plot(t, vys, 'g*', label="vy")
  axang = axxy.twinx()
  axang.plot(t, yaws, 'c', label="yaw")
  axang.plot(t, omegas, 'y', label="omega")
  axang.plot(t, heels, 'r', label="heel")
  axang.plot(t, thetacs, 'm', label="Leeway")
  axang.plot(t, thetaws, 'k', label="Apparent Wind")
  axxy.legend(loc='upper left')
  axang.legend(loc='upper right')
  axxy.grid()
  axang.grid()

  xs, ys, vxs, vys, yaws, omegas, heels, thetacs, vcs, thetaws, vws,\
      deltasopt, deltaropt = sim.Run(
      wind, v0, omega0, heel0, control.ControlMaxForce, dt, niter, debugf=forces)
  forces.UpdateZero()

  plt.figure()
  plt.plot(xs, ys, 'o')
  plt.title("Overall X/Y")
  plt.savefig('circles_sim_starboard_turn.eps')

  plt.figure()
  axxy = plt.subplot(111, sharex=axxy, sharey=axxy)
  axxy.plot(t, xs, 'b', label="x")
  axxy.plot(t, ys, 'g', label="y")
  axxy.plot(t, vxs, 'b*', label="vx")
  axxy.plot(t, vys, 'g*', label="vy")
  axxy.plot(t, vcs, 'm--', label="vc")
  axxy.plot(t, vws, 'k--', label="vw")
  axxy.set_ylim([-2, 2])
  axang2 = axxy.twinx()
  axang2.get_shared_y_axes().join(axang, axang2)
  axang2.plot(t, yaws, 'c', label="yaw")
  axang2.plot(t, omegas, 'y', label="omega")
  axang2.plot(t, heels, 'r', label="heel")
  axang2.plot(t, thetacs, 'm', label="Leeway")
  axang2.plot(t, thetaws, 'k', label="Apparent Wind")
  axxy.legend(loc='upper left')
  axang2.legend(loc='upper right')
  axang2.set_ylim([-np.pi, np.pi])
  axxy.grid()
  axang2.grid()

  plt.figure()
  axopts = plt.subplot(111, sharex=axxy)
  plt.title("Controller values for deltas, deltar")
  axopts.plot(t, deltasopt, 'b', label="Sail Opt")
  axopts.plot(t,
      [-Norm(ds + w + np.pi) for ds, w in zip(deltasopt, thetaws)],
      'r', label="Sail Angle of Attack")
  axoptr = axopts.twinx()
  axoptr.plot(t, deltaropt, 'g', label="Rudder Opt")
  axoptr.legend(loc='upper right')
  axopts.legend(loc='upper left')
  plt.grid()

  plt.figure()
  axtau = plt.subplot(111, sharex=axxy)
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
  axflon = plt.subplot(211, sharex=axxy)
  plt.title('Longitudinal Forces')
  axflat = plt.subplot(212, sharex=axxy)
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

  plt.show()
