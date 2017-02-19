#include "rudder.h"
#include "control/python/rudder_gains.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace sailbot {
namespace control {

namespace {
  template <typename Scalar>
  Scalar GetYaw(Eigen::Matrix<Scalar, 3, 3> rotMat) {
    Scalar yaw = std::atan2(rotMat(1, 0), rotMat(0, 0));
    return yaw;
  }
}

RudderController::RudderController()
    : Node(dt_) /*, K_(gains::RudderGains::K)*/, omega(0), psi(0), omegay(0),
      gammab(0), vlon(0), vlat(0), heel(0), gammar(0), omegar(0), chat(0),
      cmd_queue_("rudder_cmd", true),
      cmd_msg_(AllocateMessage<msg::RudderCmd>()) {
  RegisterHandler<msg::BoatState>("boat_state",
                                  [this](const msg::BoatState &state) {
    //Eigen::Quaternionf quat(state.orientation().w(), state.orientation().x(),
    //                        state.orientation().y(), state.orientation().z());
    //Eigen::Matrix<double, 3, 3> rotMat = quat.toRotationMatrix();
    //std::unique_lock<std::mutex> lck(state_access_);
    /*
    X_(0, 0) = GetYaw(rotMat);
    X_(1, 0) = (rotMat * state.omega())(2, 0);
    X_(2, 0) = state.internal().rudder();
    X_(3, 0) = state.internal().rudderdot();
    */

    omega = state.omega().z();
    psi = state.euler().yaw();
    heel = state.euler().roll();
    double spsi = std::sin(psi);
    double cpsi = std::cos(psi);
    vlat = state.vel().x() * cpsi + state.vel().y() * spsi;
    vlon = -state.vel().x() * spsi + state.vel().y() * cpsi;
    double sheel = std::sin(heel);
    double cheel = std::cos(heel);
    omega = state.omega().z() * cheel + state.omega().y() * sheel;

    double gamma = std::atan2(state.vel().y(), state.vel().x());
    gammab = gamma - psi;
    // TODO(james): Remove assumption that omegay == omega
    omegay = omega.load();
  });
  RegisterHandler<msg::HeadingCmd>("heading_cmd",
                                   [this](const msg::HeadingCmd &cmd) {
    //std::unique_lock<std::mutex> lck(state_access_);
    gammar = cmd.heading();
    omegar = 0; // TODO(james): Parameterize
  });
}

void RudderController::Iterate() {
  //std::unique_lock<std::mutex> lck(state_access_);
  //double U = K_ * X_;
  // See "Routing and course control of an autonomous sailboat" (2015)
  // Constants (TODO: Pull out)
  constexpr double J = 5; // Yaw moment of inertia kg * m^2
  constexpr double sigmar = 15; // kg, Hydrodynamic constant = 0.5 * rhowater *
                                // Area * lever_arm * C_liftdrag
  constexpr double Kwd = 0.1; // rad/s, Gain for calculated desired omegad
  constexpr double Kp = 10; // rad, proportional constant
  constexpr double Kd = 1; // s^-1, derivative constant
  constexpr double Kiw = 0.5; // s^-2, constant for calculating bias
  constexpr double epsilon = 1e-2; // Dimensionless, for equation (9) in paper
  constexpr double vcrit = 0.5; // m/s, Critical vlon to switch between heading/course
  constexpr double lambda = 20; // Constant for controlling rate of switching

  double vnorm = vlat * vlat + vlon * vlon;
  double speed = std::sqrt(vnorm);
  double deltar = 0;
  if (speed > 1e-2) {
    // Do smooth switching. Ignore yBdot and omegay switching, just do yB (ie, gammab)
    // TODO(james): Check math
    double X = vlat / speed / (1 + std::exp(-lambda * (vlon - vcrit)));
    gammab = std::atan2(X, std::sqrt(1 - X * X));

    double gamma = psi + gammab;

    // Inputs/useful numbers
    double yerr = gamma - gammar;

    // See equation(9)
    double omegad = -omegay + omegar -
                    Kwd * std::sin(yerr) / std::max(1 + std::cos(yerr), epsilon);
    // omegaddot = -omegaydot + omegardot - Kwd * yerrdot / (1 + cos(yerr))
    double omegaddot = 0; // TODO(james): Actually calculate
    // See equation(7)
    double u = omegaddot - Kp * std::sin(yerr) - Kd * (omega - omegad) - chat;
    double chatdot = Kiw * (omega - omegad); // See equation(8)
    chat += chatdot * dt_;
    LOG(INFO) << "Chat: " << chat;

    deltar = -0.5 *
             std::asin(std::max(
                 -1., std::min(1., J * u / (sigmar * vnorm * std::cos(heel)))));
  }
  cmd_msg_->set_pos(deltar);
  cmd_queue_.send(cmd_msg_);
}

}  // namespace control
}  // namespace sailbot
