#pragma once
#include <Eigen/Core>
#include <mutex>
#include "util/node.h"

namespace sailbot {
namespace control {

class RudderController : public Node {
  template <int M, int N>
  using Matrix = Eigen::Matrix<double, M, N>;
 public:
  RudderController();
  void set_goal_yaw(float yaw) {
    R_ << yaw, 0, 0, 0;
  }
 private:
  static constexpr float dt_ = 0.01;
  void Iterate() override;

  void CalcDARE(int steps) {}

  const Matrix<1, 4> K_;
  std::mutex state_access_;
  // State vector: [Yaw, Yaw-dot, Rudder, Rudder-dot]
  Matrix<4, 1> X_; // Current state
  // System inputs are approx rudder torque
  Matrix<4, 1> R_; // Current goal
  Matrix<3, 1> Y_; // Current measurement

  // Various State info for the paper-based controller
  // Note: I use y as a substitute for gammma and w as a substitute for omega in
  // some spots.
  std::atomic<double> omega; // rad/s, boat yaw velocity
  std::atomic<double> psi; // rad, boat yaw
  std::atomic<double> omegay; // rad/s, rotational velocity of leeway angle (dgammab / dt)
  std::atomic<double> gammab; // rad, leeway angle (ie, angle the boat is going relative to the boat)
  std::atomic<double> vlon; // m/s, Longitudinal (forwards) velocity of boat
  std::atomic<double> vlat; // m/s, Lateral (sideways) velocity of boat
  std::atomic<double> heel; // rad, heel angle (0=vertical)
  // Goal information for paper-based controller
  std::atomic<double> gammar; // rad, Goal course angle
  std::atomic<double> omegar; // rad/s, rotational velocity of goal course angle
  // Persistent state from controller
  double chat; // m / s^2, unknown perturbations of boat

  // Queues for sending out
  ProtoQueue<msg::RudderCmd> cmd_queue_;
  msg::RudderCmd* cmd_msg_;
};

}  // namespace control
}  // namespace sailbot
