#pragma once
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include "util.h"
#include <iostream>
#include "ipc/queue.hpp"
#include "sim/sim_debug.pb.h"

class SimulatorDynamics {
 public:
  typedef Eigen::Vector3d Vector3d;
  typedef Eigen::Matrix3d Matrix3d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  virtual std::pair<Vector6d, Matrix3d> Update(double sdot, double rdot) = 0;
  virtual Vector3d get_x() = 0;
  virtual Vector3d get_v() = 0;
  virtual Matrix3d get_RBI() = 0;
  virtual Vector3d get_omega() = 0;
  virtual double get_deltas() = 0;
  virtual double get_deltar() = 0;
  virtual void set_wind(Vector3d wind) = 0;
};

class TrivialDynamics : public SimulatorDynamics {
 public:
  // States:
  // heel, yaw, vx, vy, omega[wx,wy,wz], deltas, deltab (ballast), deltabdot
  // Notes: omega has 3 elements but is 2-DOF
  static constexpr int kNumXStates = 10;
  // Inputs:
  // deltasdot, deltabddot
  static constexpr int kNumUInputs = 2;
  typedef Eigen::Matrix<double, kNumXStates, 1> VectorStates;
  typedef Eigen::Matrix<double, kNumUInputs, 1> VectorInputs;

  TrivialDynamics(float _dt);
  std::pair<Vector6d, Matrix3d> Update(double sdot, double rdot);
  Vector3d get_x() { return x;}
  Vector3d get_v() { return v; }
  Matrix3d get_RBI() { return RBI; }
  Vector3d get_omega() { return omega; }
  double get_deltas() { return deltas; }
  double get_deltar() { return deltar; }
  void set_wind(Vector3d wind) { this->wind = wind; }
  double get_alpha_sail() {
    Vector3d wa = RBI.transpose() * (wind - v);
    return norm_angle(std::atan2(-wa(1), -wa(0)) - deltas);
  }

  // Note: CalcXdot does make use of most parameters that aren't explicitly
  // mentioned as part of X/U, although it assumes that those are all constant.
  // CalcXdot WILL butcher the state of this.
  VectorStates CalcXdot(double /*t*/, VectorStates X, VectorInputs U);

 private:
  // All forces/torques calculated in frame of sailboat body.
  Vector3d SailForces();
  Vector3d RudderForces();
  Vector3d KeelForces();
  Vector3d HullForces();
  // For some torques, pass in the result of the XXXForces() function.
  Vector3d SailTorques(const Vector3d& f);
  Vector3d RudderTorques(const Vector3d& f);
  Vector3d KeelTorques(const Vector3d& f);
  Vector3d HullTorques();
  Vector3d RightingTorques(double heel); // Buoyancy/Gravity/Ballast.

  Vector3d AeroForces(Vector3d v, const float delta, const float rho,
                      const float A, const float mindrag = .05,
                      const float maxdrag = 1.5, const float maxlift = 1.5);

  const float dt;
  // Note on Coord systems:
  // The origin for the body frame of the boat is located on the vertical plane
  // cutting straight down the long axis of the hull. Intuitively, it should be
  // roughly on the deck vertically and in the middle of the boat front-to-back.
  const double rr; // X position of back of rudder post wrt origin (generally negative)
  const double rs; // X position of mast wrt origin of boat
  const double rk; // X position of keel wrt origin of boat
  const double ls; // Lateral distance of CoE of sail from mast (sort of "radius" of sail)
  const double lr; // Lateral distance of CoE of rudder from rotation ("radius" of rudder)
  const double hs; // Vertical distance of CoE of sail from origin (sort of "height" of sail)
  const double hr; // Vertical distance of CoE of rudder from origin ("height" of rudder). Will be a negative number.
  const double hk;  // Vertical distance of CoE of keel from origin ("height" of
                    // keel). Will be a negative number.
  const double lm;  // Radius of movable ballast (m).
  const double mm;  // Mass of movable ballast (kg).
  const Vector3d CoM; // Center of mass
  const float mass;
  Matrix3d J;

  double deltas, deltar, deltab; // Sail, Rudder, movable Ballast
  Vector3d x, v;
  double yaw, heel;
  Matrix3d RBI;
  Vector3d omega; // w.r.t. hull frame
  Vector3d wind;

  sailbot::ProtoQueue<sailbot::msg::SimDebugMsg> debug_queue_;
};

class SimulatorSaoud2013 : public SimulatorDynamics {
 public:
  // See Hadi Saoud, Minh-Duc Hua, Frederic Plumet, and Faiz Ben Amar. Modeling and control design of a robotic sailboat. In Robotic Sailing 2013, pages 95–110. Springer, 2014.
  // Outstanding questions (TODO(james)):
  // -What is the deal with the component of the lift pointing in the direction of the wind?

  SimulatorSaoud2013(float _dt=0.001/*s*/);

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
  Vector3d is() { return RBI*isB(); }
  Vector3d js() { return RBI*jsB(); }
  Vector3d ks() { return RBI*ksB(); }
  // Rudder frame information. Same terminology as sail.
  Vector3d xrB() {
    return Vector3d(-rr - lr * std::cos(deltar), -lr * std::sin(deltar), hr);
  }
  Vector3d irB() { return Vector3d(std::cos(deltar), std::sin(deltar), 0); }
  Vector3d jrB() { return Vector3d(-std::sin(deltar), std::cos(deltar), 0); }
  Vector3d krB() { return Vector3d(0, 0, 1); }
  Vector3d xr() { return TBI(xrB()); }
  Vector3d ir() { return RBI*irB(); }
  Vector3d jr() { return RBI*jrB(); }
  Vector3d kr() { return RBI*krB(); }

  // Derivative of sail/rudder/keel states
  Vector3d GGs() { return RBI * xsB(); }
  Vector3d vs() { return v + omega.cross(GGs()) - ls * deltasdot * js(); }
  Vector3d vas() { return vs() - vw; }

  Vector3d GGr() { return RBI * xrB(); }
  Vector3d vr() { return v + omega.cross(GGr()) - lr * deltardot * jr(); }
  Vector3d var() { return vr() - vc; }

  Vector3d GGk() { return RBI * Vector3d(0, 0, hk); }
  Vector3d vk() { return v + omega.cross(GGk()); }
  Vector3d vak() { return vk() - vc; }

  Vector3d omegaB() { return RBI.inverse() * omega; }
  Vector3d vB() { return RBI.inverse() * v; }

  // Sail/Rudder/Keel angle of attack
  double alphas() {
    Vector3d v = vas();
    double d = std::sqrt(std::pow(v.dot(is()), 2) + std::pow(v.dot(ks()), 2));
    if (std::abs(d) < 1e-3) return 0;
    else return std::atan(-v.dot(js()) / d);
  }
  double alphar() {
    Vector3d v = var();
    double d = std::sqrt(std::pow(v.dot(ir()), 2) + std::pow(v.dot(kr()), 2));
    if (std::abs(d) < 1e-3) return 0;
    else return std::atan(-v.dot(jr()) / d);
  }
  double alphak() {
    Vector3d v = RBI * vak();
    double d = std::sqrt(std::pow(v(0, 0), 2) + std::pow(v(2, 0), 2));
    if (std::abs(d) < 1e-3) return 0;
    else return std::atan(-v(1, 0) / d);
  }

  // Useful constant...
  // Aerodynamic forces (Fs) and torque (taus)
  Vector3d Fs() {
    double as = alphas();
    double CL = CsL(as);
    double CD = CsD(as);
    //std::cout << "Sail CL, Cd, as: " << CL << " " << CD << " " << as << std::endl;
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
    //std::cout << "Rudder CL, Cd, as: " << CL << " " << CD << " " << ar << std::endl;
    double lambdar = .5 * rhowater * Sr;
    Vector3d Fr = -lambdar * (CD - CL * std::tan(ar)) * va.norm() * va +
                  lambdar * CL / std::cos(ar) * va.squaredNorm() * jr();
    return Fr;
  }
  Vector3d taur() { return GGr().cross(Fr()); }
  Vector3d Fk() {
    double ak = alphak();
    double CL = CkL(ak);
    double CD = CkD(ak);
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
    return va.cwiseAbs2().cwiseProduct(c1) + va.cwiseProduct(c2);
  }
  Vector3d taud() {
    Vector3d w = omegaB();
    return w.cwiseAbs2().cwiseProduct(c3) + w.cwiseProduct(c4);
  };

  // Restoring forces from Gravity/Buoyancy.
  Vector3d FBuoy() { return nabla * rhowater * g * k0(); }
  // TODO(james): Dynamically set nabla to prevent blowup from a constant
  // vertical force; with the current nabla, Fres() will evaluate to 0 anyways.
  Vector3d Fres() { return Vector3d::Zero();}//-m0 * g * k0() + FBuoy(); }
  Vector3d taures() { return (RBI * B_B).cross(FBuoy()); }

  // Coefficients for Equations of motion
  Matrix6d MRB() {
    Matrix6d out = Matrix6d::Zero();
    out(0, 0) = m0;
    out(1, 1) = m0;
    out(2, 2) = m0;
    out.block(3, 3, 3, 3) = J0;
    return out;
  }
  Matrix6d CRB() {
    Matrix6d out = Matrix6d::Zero();
    Vector3d wb = omegaB();
    out.block(0, 0, 3, 3) = m0 * Skew(wb);
    out.block(3, 3, 3, 3) = -Skew(J0 * wb);
    return out;
  }
  Matrix6d CA() {
    Matrix6d out = Matrix6d::Zero();
    Matrix3d A11 = MA.block(0, 0, 3, 3);
    Matrix3d A12 = MA.block(0, 3, 3, 3); // Approximately 0
    Matrix3d A21 = MA.block(3, 0, 3, 3); // Approximately 0
    Matrix3d A22 = MA.block(3, 3, 3, 3);
    out.block(3, 3, 3, 3) = -Skew(A21 * vB() + A22 * omegaB());
    out.block(0, 3, 3, 3) = -Skew(A11 * vB() + A12 * omegaB());
    out.block(3, 0, 3, 3) = out.block(0, 3, 3, 3);
    return out;
  }
  Matrix6d MT() { return MRB() + MA; }
  Matrix6d CT() { return CRB() + CA(); }
  Vector6d nudot() {
    Matrix6d MTinv = MT().inverse();
    Vector3d Fnet = Fd() + Fres() + Fs() + Fr() + Fk();
    /*
    std::cout << "Fd: " << Fd().transpose() << " Fres: " << Fres().transpose()
              << " Fs: " << Fs().transpose() << " Fr: " << Fr().transpose()
              << " Fk: " << Fk().transpose() << std::endl;
              */
    Vector3d taunet = taud() + taures() + taus() + taur() + tauk();
    /*
    std::cout << "td: " << taud().transpose()
              << " tres: " << taures().transpose()
              << " ts: " << taus().transpose() << " tr: " << taur().transpose()
              << " tk: " << tauk().transpose()
              << " tnetB: " << (RBI * taunet).transpose() << std::endl;
              */
    Vector3d FnetB = RBI.inverse() * Fnet;
    Vector3d taunetB = RBI.inverse() * taunet;
    Vector6d forces;
    forces << FnetB, taunetB;
    Vector6d nu;
    nu << vB(), omegaB();
    return MTinv * (forces - CT() * nu);
  }
  void Update() {
    deltas += deltasdot * dt;
    deltar += deltardot * dt;
    /*
    std::cout << "dr: " << deltar << " ds: " << deltas
              << " vw: " << vw.transpose() << " vas: " << vas().transpose()
              << " vwa: " << (vw - v).transpose() << std::endl;
              */
    Vector6d deltanu = nudot() * dt;
    Vector3d vbody = vB() + deltanu.block(0, 0, 3, 1);
    Vector3d omegabody = omegaB() + deltanu.block(3, 0, 3, 1);
    Matrix3d wx = Skew(omegabody);
    // Do actual update of v, omega:
    v = RBI * vbody;
    omega = RBI * omegabody;

    // Do update of x, RBI:
    x += v * dt;
    RBI += RBI * wx * dt;
    RBI = Orthogonalize(RBI);
  }

  std::pair<Vector6d, Matrix3d> Update(double sdot, double rdot) override {
    deltasdot = sdot;
    deltardot = rdot;
    Update();
    Vector6d nu;
    nu << x, v;
    return {nu, RBI};
  }

  Vector3d get_x() override { return x; }
  Vector3d get_v() override { return v; }
  Vector3d get_omega() override { return omega; }
  Matrix3d get_RBI() override { return RBI; }
  double get_deltas() override { return deltas; }
  double get_deltar() override { return deltar; }
  void set_wind(Vector3d wind) { vw = wind; }

 private:
  static constexpr double g = 9.8; // m/s^2
  const double dt; // Timestep.
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
  const double rhoair; // Density of Air
  const double rhowater; // Density of Water
  double Ss, Sr, Sk; // Surface area of the sail, rudder, and keel
  // Convenient constants for all the lift/drag coefficients, and how to interpret them
  const double c0s, c1s, c0r, c1r, c0k, c1k;
  double CDparam(double c0, double c1, double alpha) {
    return c0 + 2 * c1 * std::pow(std::sin(alpha), 2);
  }
  double CLparam(double c1, double alpha) {
    return (std::abs(alpha) < 1) * c1 * std::sin(2 * alpha);
  }
  // Lift & Drag coefficients of sail
  double CsL(double alpha) { return CLparam(c1s, alpha); }
  double CsD(double alpha) { return CDparam(c0s, c1s, alpha); }
  // Lift & Drag coefficients of rudder
  double CrL(double alpha) { return CLparam(c1r, alpha); }
  double CrD(double alpha) { return CDparam(c0r, c1r, alpha); }
  // Lift & Drag coefficients of keel
  double CkL(double alpha) { return CLparam(c1k, alpha); }
  double CkD(double alpha) { return CDparam(c0k, c1k, alpha); }
  /*const*/ Vector3d c1, c3; // Coefficients for quadratic term of hull resistance force/torque.
  /*const*/ Vector3d c2, c4; // Coefficients for linear term of hull resistance force/torque
  const double m0; // Boat mass
  double nabla; // Water volume displacement
  Matrix3d J0; // Boat Moment of Inertia
  Matrix6d MA; // ``Added'' inertia matrix.
  Vector3d /*M,*/ B_B; // Boat metacenter and center of buoyancy, wrt body frame.
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

