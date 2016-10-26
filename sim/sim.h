#include <eigen3/Eigen/Core>
#include "util.h"

class SimulatorSaoud2013 {
  using Eigen::Vector3d;
  using Eigen::Matrix3d;
  typedef Eigen::Matrix<double, 3, 4> Matrix34d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  // See Hadi Saoud, Minh-Duc Hua, Frederic Plumet, and Faiz Ben Amar. Modeling and control design of a robotic sailboat. In Robotic Sailing 2013, pages 95–110. Springer, 2014.
  // Outstanding questions (TODO(james)):
  // -What is the deal with the component of the lift pointing in the direction of the wind?

  SimulatorSaoud2013();

  // Inertial frame unit vectors
  Vector3d i0() { return Vector3d(1,0,0); }
  Vector3d j0() { return Vector3d(0,1,0); }
  Vector3d k0() { return Vector3d(0,0,1); }
  // Body frame information:
  // Origin at g, i points forwards along boat, j points to port in plane of
  // deck, k points up parallel to mast.
  Vector3d i() { return RBI.col(0); }
  Vector3d j() { return RBI.col(1); }
  Vector3d k() { return RBI.col(2); }
  // Sail frame information:
  // Origin at xs, ks points straight up parallel to the mast.
  // is and js are parallel to the plane of the deck. is is in the plane of the
  // sail and points inwards towards the mast; js is perpendicular to the plane
  // of the sail and its direction fulfills the requirements of a right-handed
  // coordinate system. Provides vectors relative to body frame (hull).
  // TSB = [RSB xs] = [isB jsB ksB xsB]
  Vector3d xsB() {
    return Vector3d(rs - ls * std::cos(deltas), -ls * std::sin(deltas), hs);
  }
  Vector3d isB() { return Vector3d(std::cos(deltas), std::sin(deltas), 0); }
  Vector3d jsB() { return Vector3d(-std::sin(deltas), std::cos(deltas), 0); }
  Vector3d ksB() { return Vector3d(0, 0, 1); }
  Vector3d xs() { return TBI(xsB()); }
  Vector3d is() { return TBI(isB()); }
  Vector3d js() { return TBI(jsB()); }
  Vector3d ks() { return TBI(ksB()); }
  // Rudder frame information. Same terminology as sail.
  Vector3d xrB() {
    return Vector3d(-rr - lr * std::cos(deltar), -lr * std::sin(deltar), hr);
  }
  Vector3d irB() { return Vector3d(std::cos(deltar), std::sin(deltar), 0); }
  Vector3d jrB() { return Vector3d(-std::sin(deltar), std::cos(deltar), 0); }
  Vector3d krB() { return Vector3d(0, 0, 1); }
  Vector3d xr() { return TBI(xrB()); }
  Vector3d ir() { return TBI(irB()); }
  Vector3d jr() { return TBI(jrB()); }
  Vector3d kr() { return TBI(krB()); }

  // Derivative of sail/rudder/keel states
  Vector3d GGs() { return RBI * xsB(); }
  Vector3d vs() { return v + omega.cross(GGs) - ls * deltasdot * js(); }
  Vector3d vas() { return vs() - vw; }

  Vector3d GGr() { return RBI * xrB(); }
  Vector3d vr() { return v + omega.cross(GGr) - lr * deltardot * jr(); }
  Vector3d var() { return vr() - vc; }

  Vector3d GGk() { return RBI * Vector3d(0, 0, hk); }
  Vector3d vk() { return v + omega.cross(GGk); }
  Vector3d vak() { return vk() - vc; }

  Vector3d omegaB() { return RBI.inverse() * omega; }
  Vector3d vB() { return RBI.inverse() * v; }

  // Sail/Rudder/Keel angle of attack
  double alphas() {
    Vector3d v = vas();
    return std::atan(-v.dot(js()) / std::sqrt(std::pow(v.dot(is()), 2) +
                                              std::pow(v.dot(ks()), 2)));
  }
  double alphar() {
    Vector3d v = var();
    return std::atan(-v.dot(jr()) / std::sqrt(std::pow(v.dot(ir()), 2) +
                                              std::pow(v.dot(kr()), 2)));
  }
  double alphak() {
    Vector3d v = RBI * vak();
    return std::atan(-v(1, 0) / std::sqrt(std::pow(v(0, 0), 2) +
                                          std::pow(v(2, 0), 2)));
  }

  // Useful constant...
  // Aerodynamic forces (Fs) and torque (taus)
  Vector3d Fs() {
    double as = alphas();
    double CL = CsL(as);
    double CD = CsD(as);
    Vector3d va = vas();
    double lambdas = .5 * rhoair * Ss;
    Vector3d Fs = -lambdas * (CD - CL * std::tan(as)) * va.norm() * va +
                  lambdas * CL / std::cos(as) * va.squaredNorm() * js();
    return Fs;
  }
  Vector3d taus() { return GGs().cross(Fs()); }
  Vector3d Fr() {
    double ar = alphar();
    double CL = CrL(ar);
    double CD = CrD(ar);
    Vector3d va = var();
    double lambdar = .5 * rhowater * Sr;
    Vector3d Fr = -lambdar * (CD - CL * std::tan(ar)) * va.norm() * va +
                  lambdar * CL / std::cos(ar) * va.squaredNorm() * jr();
    return Fr;
  }
  Vector3d taur() { return GGr().cross(Fr()); }
  Vector3d Fk() {
    double ak = alphak();
    double CL = CrL(ak);
    double CD = CrD(ak);
    Vector3d va = vak();
    double lambdak = .5 * rhowater * Sk;
    Vector3d Fk = -lambdak * (CD - CL * std::tan(ak)) * va.norm() * va +
                  lambdak * CL / std::cos(ak) * va.squaredNorm() * j();
    return Fk;
  }
  Vector3d tauk() { return GGk().cross(Fk()); }

  // Hydrodynamic resistances of the hull/waves, approximated quadratically.
  Vector3d Fd() {
    Vector3d va = RBI.inverse() * (v - vc);
    return va.transpose() * c1 * va + va.dot(c2);
  }
  Vector3d taud() {
    Vector3d w = omegaB();
    return w.transpose() * c3 * w + w.dot(c4);
  };

  // Restoring forces from Gravity/Buoyancy.
  Vector3d FBuoy() { return nabla * rhowater * k0(); }
  Vector3d Fres() { return -m0 * g * k0() + FBuoy(); }
  Vector3d taures() { return FBuoy().cross(B); }

  // Coefficients for Equations of motion
  Matrix6d MRB() {
    Matrix6d out = Matrix6d::Zero();
    out(0, 0) = m0;
    out(1, 1) = m0;
    out(2, 2) = m0;
    out.block(3, 3, 3, 3) = J0;
    return out;
  };
  Matrix6d CRB() {
    Matrix6d out = Matrix6d::Zero();
    out.block(0, 0, 3, 3).diagonal() = m * omegaB();
    out.block(3, 3, 3, 3).diagonal() = -J0 * omegaB();
    return out;
  }
  Matrix6d CA() {
    Matrix6d out = Matrix6d::Zero();
    Matrix3d A11 = MA.block(0, 0, 3, 3);
    Matrix3d A12 = MA.block(0, 3, 3, 3); // Approximately 0
    Matrix3d A21 = MA.block(3, 0, 3, 3); // Approximately 0
    Matrix3d A22 = MA.block(3, 3, 3, 3);
    out.block(3, 3, 3, 3).diagonal() = -(A21 * vB() + A22 * omegaB());
    out.block(0, 3, 3, 3).diagonal() = -(A11 * vB() + A12 * omegaB());
    out.block(3, 0, 3, 3).diagonal() = out.block(0, 3, 3, 3);
    return out;
  }
  Matrix6d MT() { return MRB() + MA; }
  Matrix6d CT() { return CRB() + CA(); }
  Vector6d nudot() {
    Matrix6d MTinv = MT().inverse();
    Vector3d Fnet = Fd() + Fres() + Fs() + Fr() + Fk();
    Vector3d taunet = taud() + taures() + taus() + taur() + tauk();
    Vector3d FnetB = RBI * Fnet;
    Vector3d taunetB = RBI * taunet;
    Vector6d forces;
    forces << FnetB, taunetB;
    Vector6d nu;
    nu << vB(), omegaB();
    return MT().inverse() * (forces - CT() * nu);
  }
  void Update() {
    Vector6d vdot = nudot();
    Vector3d vB = vB() + vdot.block(0, 0, 3, 1);
    Vector3d omegaB = omegaB() + vdot.block(3, 0, 3, 1);
    Matrix3d wx;
    wx.diagonal() = omegaB;
    // Do actual update of v, omega:
    v = RBI * vB;
    omega = RBI * omegaB;

    // Do update of x, RBI:
    x += v * dt;
    RBI += RBI * wx * dt;
  }

 private:
  double dt; // Timestep.
  // Parameters, same as Saoud article. All in SI units.
  const double ls; // Lateral distance of CoE of sail from mast (sort of "radius" of sail)
  const double lr; // Lateral distance of CoE of rudder from rotation ("radius" of rudder)
  const double hs; // Vertical distance of CoE of sail from G (sort of "height" of sail)
  const double hr; // Vertical distance of CoE of rudder from G ("height" of rudder). Will be a negative number.
  const double hk; // Vertical distance of CoE of keel from G ("height" of keel). Will be a negative number.
  double deltas; // Angle of sail from mast. 0=all the way in, +pi/2=boom on starboard side.
  double deltasdot; // Time derivative of deltas.
  double deltar; // Angle of rudder. 0=straight, +pi/2=boat is turning starboard
  double deltardot; // Time derivative of deltar.
  const double rr; // Distance from back of boat to CoM of boat.
  const double rs; // Distance from CoM of boat to mast
  //double alphas; // Sail's angle of attack
  double alphar; // Rudder's angle of attack
  double alphak; // Keel's angle of attack
  const double rhoair; // Density of Air
  const double rhowater; // Density of Water
  double Ss, Sr, Sk; // Surface area of the sail, rudder, and keel
  // Convenient constants for all the lift/drag coefficients, and how to interpret them
  const double c0s, c1s, c0r, c1r, c0k, c1k;
  double CDparam(double c0, double c1, double alpha) {
    return c0 + 2 * c1 * std::pow(std::sin(alpha), 2);
  }
  double CLparam(double c1, double alpha) { return c1 * std::sin(2*alpha); }
  // Lift & Drag coefficients of sail
  double CsL(double alpha) { return CLparam(c1s, alpha); }
  double CsD(double alpha) { return CDparam(c0s, c1s, alpha); }
  // Lift & Drag coefficients of rudder
  double CrL(double alpha) { return CLparam(c1r, alpha); }
  double CrD(double alpha) { return CDparam(c0r, c1r, alpha); }
  // Lift & Drag coefficients of keel
  double CkL(double alpha) { return CLparam(c1k, alpha); }
  double CkD(double alpha) { return CDparam(c0k, c1k, alpha); }
  const Matrix3d c1, c3; // Coefficients for quadratic term of hull resistance force/torque. Generally diagonal matrices.
  const Vector3d c2, c4; // Coefficients for linear term of hull resistance force/torque
  const double m0; // Boat mass
  double nabla; // Water volume displacement
  Matrix3d J0; // Boat Moment of Inertia
  Matrix6d MA; // ``Added'' inertia matrix.
  Vector3d /*M*/, B; // Boat metacenter and center of buoyancy, wrt body frame.
  // Non-written variables:
  // G: Boat CoM, wrt inertial frame
  // Gs, Gr, Gk: Centers of pressures of sail/rudder/keel, wrt body frame
  // Frames:
  // Inertial fixed frame, Body frame (fixed to hull/G), Sail, and Rudder.
  Vector3d x; // Location of G
  Vector3d v; // Vel. of G wrt inertial frame
  //Vector3d vas, var, vak; // Vel. of G/Gs/Gr/Gk wrt Wind
  Vector3d vw; // True Wind Velocity
  Vector3d vc; // True Water Current
  // Reminder on Rotation matrices:
  // RXY denotes orientation of X wrt Y, so xy = Rxy * xx.
  Matrix3d RBI; // The unit vectors of the boat in terms of inertial frame
  Vector3d omega; // Boat angular velocity

  Vector3d TBI(const Vector3d& p) { return Trans(p, RBI, x); }
};


SimulatorSaoud2013::SimulatorSaoud2013() : dt(0.01/*s*/),
                                           ls(.5/*m*/),
                                           lr(.05/*m*/),
                                           hs(1.5/*m*/),
                                           hr(-.3/*m*/),
                                           hk(-.6/*m*/),
                                           deltas(0/*rad*/),
                                           deltasdot(0/*rad/s*/),
                                           deltar(0/*rad*/),
                                           deltardot(0/*rad/s*/),
                                           rr(1/*m*/),
                                           rs(.5/*m*/),
                                           alphar(0/*rad*/),
                                           alphak(0/*rad*/),
                                           rhoair(1.225/*kg/m^3*/),
                                           rhowater(1000/*kg/m^3*/),
                                           Ss(5/*m^2*/),
                                           Sr(.25/*m^2*/),
                                           Sk(.5/*m^2*/),
                                           c0s(.5), c1s(.5), c0r(c0s), c1r(c1s), c0k(c0s), c1k(c1s),
                                           m0(40/*kg*/),
                                           nabla(m0 / rhowater) {
  // TODO(james):
  // Not really sure what these should initialize to. They refer to the
  // resistance along the hull through the water. Definitely shouldn't be zero.
  c1 *= 0;
  c3 *= 0;
  c2 *= 0;
  c4 *= 0;

  J0 << ; // TODO(james) ?

  MA *= 0; // TODO(james): Might be able to get away with 0...
  // TODO(james): Calculations for metacenter/center of buoyancy.
  B = TBI(Vector3d(0, 0, -.5));

  x *= 0;
  v *= 0;
  vw *= 0;
  vc *= 0;
  RBI = RBI.Identity();
  omega *= 0;
}
