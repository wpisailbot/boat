#include "rudder.h"
#include "control/python/rudder_gains.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace sailbot {

namespace {
  template <typename Scalar>
  Scalar GetYaw(Eigen::Matrix<Scalar, 3, 3> rotMat) {
    Scalar yaw = std::atan2(rotMat(1, 0), rotMat(0, 0));
    return yaw;
  }
}

RudderController::RudderController() : Node(dt_), K_(gains::RudderGains::K) {
  RegisterHandler<msg::BoatState>([this](const msg::BoatState &state) {
    Eigen::Quaternionf quat(state.orientation().w(), state.orientation().x(),
                            state.orientation().y(), state.orientation().z());
    Eigen::Matrix<double, 3, 3> rotMat = quat.toRotationMatrix();
    X_(0, 0) = GetYaw(rotMat);
    X_(1, 0) = (rotMat * state.omega())(2, 0);
    X_(2, 0) = state.internal().rudder();
  });
}
}  // namespace sailbot
