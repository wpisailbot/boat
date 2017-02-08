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
 private:
  const float dt_ = 0.01;
  void Iterate() override;
  const Matrix<1, 4> K_;
  std::mutex state_mutex_; // Mutex for access to X_ and R_
  // State vector: [Yaw, Yaw-dot, Rudder, Rudder-dot]
  Matrix<4, 1> X_; // Current state
  Matrix<4, 1> R_; // Current goal
  Matrix<3, 1> Y_; // Current measurement
};

}  // namespace sailbot
