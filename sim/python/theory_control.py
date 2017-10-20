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
    atanC, _ = self.rudder.atanClCd(alphar)
    atanC = (np.pi / 2.0 - 0.05) * Sign(alphar)
    Fr, deltaFr = self.rudder.F(alphar, vc)
    gammar = Norm(atanC - thetac + np.pi)
    deltaFr = deltaFr * -1.0 # dalphar / ddeltar = -1
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
    r = np.sqrt(self.rk ** 2 + (self.hk * np.sin(heel)) ** 2)
    theta = gammak - np.arctan2(-self.hk * np.sin(heel), self.rk)
    return r * Fk * np.sin(theta) * np.cos(heel)

  def RudderTorque(self, Fr, gammar, heel, deltaFr, deltagammar, deltaheel):
    """
      Calculate yaw torque from rudder, using output from RudderForces
      Assumes self.hr is negligible.
    """
    tau = self.rr * Fr * np.sin(gammar) * np.cos(heel)
    dtaudr = self.rr * np.cos(heel) \
        * (Fr * np.cos(gammar) * deltagammar + deltaFr * np.sin(gammar))
    dtauds = -self.rr * Fr * np.sin(gammar) * np.sin(heel) * deltaheel
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
    taur, dtaurdr, dtaurds = self.RudderTorque(Fr, gammar, heel, dFrdr, dgrdr, dheelds)
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

  def RunBase(self, ts, winds, x0, v0, yaw0, omega0, heel0, deltass, deltars,
              flopsail=False, debugf=None):
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
    for i in range(1, len(ts)):
      dt = np.clip(ts[i] - ts[i - 1], 0.001, 0.2)
      wx = winds[0][i]
      wy = winds[1][i]
      deltas = deltass[i]
      deltar = deltars[i]
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

    return xs, ys, vxs, vys, yaws, omegas, heels, thetacs, vcs, thetaws, vws

  def Run(self, wind, v0, omega0, heel0, deltas, deltar, dt=0.01, niter=200, flopsail=True, debugf=None):
    wx = [wind[0]] * niter
    wy = [wind[1]] * niter
    winds = [wx, wy]
    deltass = deltas if isinstance(deltas, list) else [deltas] * niter
    deltars = deltar if isinstance(deltar, list) else [deltar] * niter
    ts = [i * dt for i in range(niter)]
    return self.RunBase(ts, winds, [0.0, 0.0], v0, 0.0, omega0, heel0,
                        deltass, deltars, flopsail=flopsail, debugf=debugf)
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

  def ClipSail(self, deltas, thetaw):
    maxsail = abs(Norm(np.pi - thetaw))
    return np.clip(deltas, 0.0 if thetaw > 0.0 else -maxsail,
                           maxsail if thetaw > 0.0 else 0.0)

  def ClipRudder(self, deltar, thetac):
    return np.clip(deltar, min(-self.maxrud - thetac, 0.0), max(self.maxrud - thetac, 0.0))

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
    print("thetaw ", thetaw, " vw ", vw)
    while deltasstep * laststep >= 0.0:# or np.isnan(taunom):
      print("Iter")
      Fs, gammas, dFsds, dgsds = self.physics.SailForces(thetaw, vw, deltas)
      print("Fs ", Fs, " gammas ", gammas, " dFsds ", dFsds, " dgsds ", dgsds)
      Fk, gammak = self.physics.KeelForces(thetac, vc)
      heel, dheelds = self.physics.ApproxHeel(Fs, gammas, Fk, gammak, dFsds, dgsds)
      Fr, gammar, dFrdr, dgrdr = self.physics.RudderForces(thetac, vc, deltar)
      taus, dtausds = self.physics.SailTorque(Fs, gammas, deltas, heel, dFsds, dgsds, dheelds)
      # Ignore the keel...
      print("Fr ", Fr, " gammar ", gammar, " dFrdr ", dFrdr, " dgrdr", dgrdr)
      taur, dtaurdr, dtaurds = self.physics.RudderTorque(Fr, gammar, heel, dFrdr, dgrdr, dheelds)
      taunet = taus + taur
      if np.isnan(taunom):
        taunom = taunet
        print("Taunom: ", taunom)
      tauerr = taunet - taunom
      print("tauerr: ", tauerr)

      dFlonds = dFsds * np.cos(gammas) - Fs * np.sin(gammas) * dgsds
      print("dFlonds: ", dFlonds, " taunet: ", taunet)

      laststep = deltasstep
      deltasstep = 0.01 * Sign(dFlonds)
      deltas += deltasstep
      dtau = dtausds * deltasstep + dtaurds * deltasstep
      print("dtau ", dtau, " dtausds ", dtausds, " dtaurds ", dtaurds, " dtaurdr ", dtaurdr)
      deltarstep = -(dtau + tauerr) / dtaurdr
      deltar += deltarstep

      clips = self.ClipSail(deltas, thetaw)
      clipr = self.ClipRudder(deltar, thetac)
      print("clips ", clips, " clipr ", clipr)
      if clips != deltas or clipr != deltar:
        print("breaking due to limit")
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


if __name__ == "__main__":
  sim = Physics()
  wind = [0.0, -3.0]
  v0 = [1.0, 0.0]
  omega0 = 0.0
  heel0 = 0.0
  deltas = 0.0
  deltar = -0.2
  dt = 0.01
  niter = 3500
  t = [dt * n for n in range(niter)]
  forces = DebugForces()
  xs, ys, vxs, vys, yaws, omegas, heels, thetacs, vcs, thetaws, vws = sim.Run(
      wind, v0, omega0, heel0, deltas, deltar, dt, niter, debugf=forces)
  forces.UpdateZero()

  control = Controller(sim)
  deltasopt = []
  deltaropt = []
  for i in range(len(thetaws)):
    print(i)
    ds = deltasopt[-1] if len(deltasopt) > 0 else deltas
    ds = abs(ds) if thetaws[i] > 0 else -abs(ds)
    dsopt, dropt = control.MaxForceForTorque(
        thetaws[i], vws[i], thetacs[i], vcs[i], ds, deltar)
    print("ds ", dsopt, " dr ", dropt)
    deltasopt.append(dsopt)
    deltaropt.append(dropt)

  plt.plot(xs, ys, 'o')
  plt.title("Overall X/Y")

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
  axxy.legend(loc='upper left')
  axang.legend(loc='upper right')
  axxy.grid()
  axang.grid()

  xs, ys, vxs, vys, yaws, omegas, heels, thetacs, _, _, _ = sim.Run(
      wind, v0, omega0, heel0, deltasopt, deltaropt, dt, niter)

  plt.figure()
  axxy = plt.subplot(111, sharex=axxy, sharey=axxy)
  axxy.plot(t, xs, 'b', label="x")
  axxy.plot(t, ys, 'g', label="y")
  axxy.plot(t, vxs, 'b*', label="vx")
  axxy.plot(t, vys, 'g*', label="vy")
  axang2 = axxy.twinx()
  axang2.get_shared_y_axes().join(axang, axang2)
  axang2.plot(t, yaws, 'c', label="yaw")
  axang2.plot(t, omegas, 'y', label="omega")
  axang2.plot(t, heels, 'r', label="heel")
  axang2.plot(t, thetacs, 'm', label="Leeway")
  axxy.legend(loc='upper left')
  axang2.legend(loc='upper right')
  axxy.grid()
  axang2.grid()

  plt.figure()
  axopt = plt.subplot(111, sharex=axxy)
  plt.title("Controller values for deltas, deltar")
  axopt.plot(t, deltasopt, label="Sail Opt")
  axopt.plot(t, deltaropt, label="Rudder Opt")
  axopt.set_ylim([-0.8, 0.8])
  axopt.legend()
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
