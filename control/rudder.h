#pragma once
#include <Eigen/Core>
#include <mutex>
#include "util/node.h"

namespace sailbot {

class RudderController : public Node {
  template <int M, int N>
  using Matrix = Eigen::Matrix<double, M, N>;
 public:
  RudderController();
  void set_goal_yaw(float yaw) {
    R_ << yaw, 0, 0, 0;
  }
 private:
  const float dt_ = 0.01;
  void Iterate() override;
  const Matrix<1, 4> K_;
  std::mutex state_mutex_; // Mutex for access to X_ and R_
  std::mutex state_access_;
  // State vector: [Yaw, Yaw-dot, Rudder, Rudder-dot]
  Matrix<4, 1> X_; // Current state
  // System inputs are approx rudder torque
  Matrix<4, 1> R_; // Current goal
  Matrix<3, 1> Y_; // Current measurement
};

}  // namespace sailbot
