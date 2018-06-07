#pragma once
// See http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
// This only seems to be an issue on the BBB, not normal laptops.
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include "rigid_wing/rigid_wing.pb.h"
#include <Eigen/Core>
#include <mutex>

namespace sailbot {
namespace control {

class ControlPhysics {
 public:
  typedef Eigen::Matrix<double, 2, 6> MatrixY;
  typedef Eigen::Matrix<double, 6, 1> MatrixBeta;
  typedef Eigen::Array<double, 6, 1> ArrayBeta;

  enum Constraint {
    kQuadratic = 0,
    kStarboard = 1,
    kPort = 2,
  };

  struct AirfoilConstants {
    double A; // Area, m^2
    double rho; // Fluid density, kg / m^3
    double C;   // Dimensionless constant corresponding to maximum available
                // force. Generally ~1
    double K;   // Dimensionless, rate at which lifting increases with angle of
                // attack.
    double drageps; // Radians, quantity used to add a bit of drag to keel,
                    // rudder, typically ~0.05-0.01
    double maxalpha; // Maximum absolute value of alpha for which this
                     // approximation is valid.
  };
  struct BoatConstants {
    // Constants for the various hydrodynamic surfaces
    AirfoilConstants sail, keel, rudder;
    // The heights of the sail, keel, and rudder Centers of Effort above the
    // origin, in meters. hk and hr would generally be negative as they are
    // typically below the origin.
    double hs, hk, hr;
    // Longitudinal positions of each sail, keel, and rudder. As with height, rr
    // will typically be negative. sail and keel depend on the setup of the
    // boat. In meters. For the sail/rudder, these are the positions of the
    // points of rotation.
    double rs, rk, rr;
    // Distance of the centers of effort from the points of rotation for the
    // sail and rudder, in meters. This means that, when the sail is fully
    // sheeted in, the center of effort of the sail will be at rs + ls meters in
    // front of the origin.
    double ls, lr;
    // Height of the center of mass above the origin, in m:
    double hb;
    // Mass of the boat, in kg
    double m;
    // Acceleration due to gravity, m / s^2
    double g;
    // Moment of Inertia of the boat about the Yaw axis, kg *m^2
    double J;
    // Damping constants in longitudinal/lateral directions and about the yaw
    // axis:
    double Blon /*kg / s*/, Blat /*kg / s*/, Bomega /*kg * m^2 / s*/;
    // Net bias in the overall torque.
    double taubias; // N-m
  };

  ControlPhysics();

  MatrixBeta beta() const {
    MatrixBeta b;
    b << consts_.Blon, consts_.Bomega, consts_.rudder.A, consts_.rs,
        consts_.taubias, 1.0;
    return b;
  }

  void IncrementBeta(const MatrixBeta &diff);
  void set_beta(const MatrixBeta &beta) {
    consts_.Blon = beta(0, 0);
    consts_.Bomega = beta(1, 0);
    consts_.rudder.A = beta(2, 0);
    consts_.rs = beta(3, 0);
    consts_.taubias = beta(4, 0);
  }

  BoatConstants consts() const { return consts_; }

  void NetForce(double thetaw, double vw, double thetac, double vc,
                double deltas, double deltar, double omega, double *Flon,
                double *Flat, double *taunet, double *newheel,
                MatrixY *Y) const;

  double RudderForTorque(double taus, double tauk, double taugoal, double heel,
                         double thetac, double vc) const;

  // Return true if a solution was found, false otherwise.
  bool GlobalMaxForceForTorque(double thetaw, double vw, double thetac,
                               double vc, double taug, Constraint constraint,
                               int nsteps, double *deltas,
                               double *deltar) const;

  double Qf = 1.0;
  double Qtaumax = 0.4;
  double Qtaueq = 0.04;

 private:

  void SailAirfoil(double alpha, double v, double *force, double *ang) const;
  void KeelAirfoil(double alpha, double v, double *force, double *ang) const;
  void RudderAirfoil(double alpha, double v, double *force, double *ang) const;

  void SailForce(double thetaw, double vw, double deltas, double *Fs,
                 double *gammas) const;
  void KeelForce(double thetac, double vc, double *Fk, double *gammak) const;
  void RudderForce(double thetac, double vc, double deltar, double *Fr,
                   double *gammar) const;
  double SailTorque(double Fs, double gammas, double deltas, double heel) const;
  double KeelTorque(double Fk, double gammak, double heel) const;
  double RudderTorque(double Fr, double gammar, double heel) const;
  double CalcHeel(double Fs, double gammas, double Fk, double gammak) const;

  double ClipRudder(double deltar, double thetac) const {
    double maxrud = consts().rudder.maxalpha;
    return std::max(std::min(deltar, maxrud - thetac), -maxrud - thetac);
  }

  BoatConstants consts_;
  ArrayBeta betamin_, betamax_;
}; // class ControlPhysics

class AdaptiveControl : public Node {
 public:
  AdaptiveControl();

  void Iterate() override;
 private:
  constexpr static float dt = 0.05;
  // Constants for updating beta; higher numbers result in more correction
  Eigen::Matrix<double, 6, 6> Kbeta;
  // Controls relative weight of state vs. derivative of state error in
  // updating beta; higher values increase weight of state over the
  // derivative.
  Eigen::Matrix<double, 2, 2> Lambda;
  // Constants for updating reference trajectory.
  // The current references are stored in yaw_ref_ and omega_ref_.
  // The reference trajectory updates as follows, at the
  // end of each iteration (to predict the next step):
  // 1) We take a weighted average of the current *ref_ and *_,
  //    to avoid excessive divergence between the reference and
  //    true trajectories. This is the new reference.
  // 2) We set a nominal expected velocity based on the difference
  //    between the reference yaw (post-weighted avg) and
  //    the commanded heading (heading_); this is clipped to
  //    avoid asking for excessive turn rates
  // 3) Set a nominal expected accel based in difference between
  //    reference omega (post avg) and the above expected velocity.
  //    This value is also clipped.
  // 4) Actually integrate the reference yaw/omega based on the
  //    reference omega and expected acceleration from step 3.

  // Value affecting weighted average in step 1,
  // where 1.0 means we ignore the true trajectory when
  // calculating the reference, and 0.0 means we ignore the
  // past reference and just compute a reference trajectory
  // a single time-step into the future.
  double Kref;

  // Maximum values for the expected velocity/accel from
  // steps 2 and 3 above. If the maximum vel is less than
  // zero, then yaw_ref_ = heading_, omega_ref_ = 0.0.
  // If maximum accel < 0, then omega_ref_ is set to
  // the expected velocity from step 2.
  double Kmax_exp_vel, Kmax_exp_acc;

  ControlPhysics::MatrixBeta Adaptor(double deltas, double deltar) const;
  /**
   * Update yaw_ref_ and omega_ref_, per the rules above.
   */
  void UpdateYawRef();
  bool Controller(double *deltas, double *deltar);

  ControlPhysics physics_;

  msg::SailCmd *sail_msg_;
  msg::RudderCmd *rudder_msg_;
  msg::BoatState *boat_state_;
  msg::ControllerConstants *consts_msg_;
  std::atomic<float> heading_;
  std::mutex boat_state_mutex_;
  std::mutex consts_mutex_;
  // State information
  std::atomic<float> yaw_{0}, thetaw_{0}, vw_{0}, thetac_{0}, vc_{0}, omega_{0};
  // The current reference yaw/omega. beta will be updated based on the
  // difference between yaw/omega and yawref/omegaref.
  std::atomic<float> yaw_ref_{0}, omega_ref_{0};
  ProtoQueue<msg::SailCmd> sail_cmd_;
  ProtoQueue<msg::RudderCmd> rudder_cmd_;
  ProtoQueue<msg::ControllerConstants> consts_queue_;
};

}  // control
}  // sailbot
