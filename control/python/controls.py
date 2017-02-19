import scipy.signal
import scipy.linalg
import numpy as np

def c2d(A, B, dt):
  a, b, _, _, _ = scipy.signal.cont2discrete((A, B, None, None), dt)
  return np.matrix(a), np.matrix(b)

def place(A, B, poles):
  return scipy.signal.place_poles(A, B, poles).gain_matrix()

def dlqr(A, B, Q, R):
  P = scipy.linalg.solve_discrete_are(A, B, Q, R)
  K = np.linalg.inv(R + B.T * P * B) * B.T * P * A
  return K

class Controller(object):

  def Discretize(self):
    self.Ad, self.Bd = c2d(self.Ac, self.Bc, self.dt)
    print("Ad:")
    print(self.Ad)
    print("Bd:")
    print(self.Bd)

  def UpdateU(self, U):
    self.X = self.Ad * self.X + self.Bd * U
    return self.X

  def CalcLQR(self):
    self.K = dlqr(self.Ad, self.Bd, self.Q, self.R)

  def DAREStep(self, X):
    """
      Performs one iteration of the DARE, given an X_t.
      Returns (X_{t-1}, K_{t})
    """
    A = self.Ad
    B = self.Bd
    Q = self.Q
    R = self.R
    AXB = A.T * X * B
    BXBRinv = np.linalg.inv(B.T * X * B + R)
    Xtm1 = Q + A.T * X * A - AXB * BXBRinv * AXB.T
    K = BXBRinv * AXB.T
    return Xtm1, K

  def CalcDARE(self, n_iters=100):
    X = self.Q
    K = None
    for _ in range(n_iters):
      X, K = self.DAREStep(X)
    self.K = K

  def FLaw(self, R):
    return self.K * (R - self.X)

  def Run(self, x_i, R, t, biasx=None):
    if biasx == None:
      biasx = np.zeros(R.shape)
    U = None
    X = x_i
    self.X = x_i
    ts = np.arange(0, t, self.dt)
    for _ in ts:
      curU = self.FLaw(R)
      if U != None:
        U = np.concatenate((U, curU), axis=1)
      else:
        U = curU
      self.X = self.UpdateU(U[:, -1])
      self.X += biasx
      X = np.concatenate((X, self.X), axis=1)
    return ts, X, U

  def WriteGains(self, fname):
    with open(fname, 'w') as f:
      mtype = "Eigen::Matrix<double, %d, %d>" % (self.K.shape[0], self.K.shape[1])
      vals = "";
      for n in self.K.A1:
        vals += repr(n) + ","
      vals = vals.strip(',')
      s = "#pragma once\n"
      s += "#include <Eigen/Core>\n"
      s += "namespace sailbot { namespace gains { namespace RudderGains {\n"
      s += "%s K = [] { %s tmp; tmp << %s; return tmp; }();\n" % (mtype, mtype, vals)
      s += "} } }"
      f.write(s)
