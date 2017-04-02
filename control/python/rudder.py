#!/usr/bin/python3
import numpy as np
import controls
from matplotlib import pyplot as plt
import sys

class Rudder(controls.Controller):
  # For the rudder control system, we will have these states:
  # boat yaw, boat yaw rate, rudder position, rudder goal bias
  # and two inputs: The goal for the rudder servo and a change in the bias
  def __init__(self, v=3):
    self.v = v # Nominal boat velocity [m/s]
    self.dt = 0.01

    self.Q = np.matrix([[25, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1e-3, 0],
                        [0, 0, 0, 0]])

    self.R = np.matrix([[30 * 1e1, 0],
                        [0, 1]])
    self.R = np.matrix([[1]])

    self.CalcAB()
    self.Discretize()
    self.CalcLQR()

  def CalcAB(self):
    Kr2y = 3 # constant for how much (r*v^2) affects yaw accel. [m^-1]
    Kyd = 1 # Dampening constant for yaw
    Kr = 2  # Rudder time constant (how rudder goal influences rudder vel)
    self.Ac = np.matrix([[0, 1, 0, 0],
                         [0, -Kyd, Kr2y * self.v * self.v, 0],
                         [0, 0, -Kr, Kr],
                         [0, 0, 0, 0]])
    self.Bc = np.matrix([[0, 0],
                         [0, 0],
                         [Kr, 0],
                         [0, 1]])
    self.Bc = self.Bc[:, 1]

  def RunAndPlot(self, R, x_i=np.zeros((4, 1)), t=10, title=None, biasx=None):
    t, X, U = self.Run(x_i, R[:4, 0], t, biasx)
    plt.figure()
    ax = plt.subplot(211)
    plt.gcf().canvas.set_window_title(title)
    plt.plot(t, U[0, :].T, label="Control input goal")
#    plt.plot(t, U[1, :].T, label="Control input bias")
    plt.legend()
    plt.subplot(212, sharex=ax)
    plt.plot(t, X[0, :-1].T, label="Yaw")
    plt.plot(t, X[1, :-1].T, label="Yaw-dot")
    plt.plot(t, X[2, :-1].T, label="Rudder")
    plt.plot(t, X[3, :-1].T, label="Rudder Bias")
    plt.legend()

  def RangeOfKs(self, vmin, vmax, dv):
    Ks = []
    vs = np.arange(vmin, vmax, dv)
    Astart = self.Ad
    for v in vs:
      self.v = v
      self.CalcAB()
      self.Discretize()
      A = np.divide(self.Ad, Astart)
      print(v)
      print(A)
      self.CalcLQR()
      Ks.append(self.K.tolist()[0])
    Ks = np.matrix(Ks)
    plt.figure()
    plt.plot(vs, Ks[:, 0], label="K0")
    plt.plot(vs, Ks[:, 1], label="K1")
    plt.plot(vs, Ks[:, 2], label="K2")
    plt.plot(vs, Ks[:, 3], label="K3")
    plt.legend()

if __name__ == "__main__":
  obj = Rudder()
  if len(sys.argv) > 1:
    obj.WriteGains(sys.argv[1])
  else:
    #obj.RunAndPlot(np.matrix([[1],[0],[0],[0]]), title="Basic Turn")
    #obj.v = 1.5
    #obj.CalcAB()
    #obj.Discretize()
    #obj.RunAndPlot(np.matrix([[1],[0],[0],[0]]), title="Innacurate (high) Velocity")
    #obj.CalcLQR()
    #obj.RunAndPlot(np.matrix([[1],[0],[0],[0]]), title="Lower Velocity")
    #obj = Rudder()
    #obj.K[1, 2] = 0
    #obj.K[1, 3] = 0
    obj.v = 6
    obj.CalcDARE(100)
    obj.CalcAB()
    obj.Discretize()
    obj.RunAndPlot(np.matrix([[1],[0],[0],[0]]), title="Innacurate (low) Velocity")
    print("K: " + repr(obj.K))
    obj.RunAndPlot(np.matrix([[1],[0],[0],[0]]),
                   title="Innacurate (low) Velocity with bias",
                   biasx=np.matrix([[0], [.1], [0], [0]]))
   # obj.K[1, :] = 0
    print("K: " + repr(obj.K))
    obj.RunAndPlot(np.matrix([[1],[0],[0],[0]]),
                   title="Innacurate (low) Velocity with bias, no correction",
                   biasx=np.matrix([[0], [.1], [0], [0]]))
    obj.CalcLQR()
    obj.RunAndPlot(np.matrix([[1],[0],[0],[0]]), title="Higher Velocity")
    obj.RangeOfKs(0.2, 10, 0.1)

    plt.show()
