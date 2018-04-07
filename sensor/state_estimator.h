#pragma once

#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include "can/can.pb.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sailbot {
namespace control {

class StateEstimator : public Node {
  typedef Eigen::Vector3d Vector3d;
  typedef Eigen::Matrix3d Matrix3d;
  typedef Eigen::Quaterniond Quaterniond;
 public:
  StateEstimator();
 private:
  void Iterate() override;

  ProtoQueue<msg::BoatState> state_queue_;
  ProtoQueue<msg::Vector3f> wind_queue_;
  ProtoQueue<msg::Vector3f> true_wind_queue_;
  // For locking access to all the state variables.
  std::mutex state_msg_mutex_;
  // Store the state in the Eigen variables until we send out the state_msg_.
  // Except for the internal state, which we will (For now) keep in state_msg_.
  Vector3d euler_angles_;
  Vector3d omega_;
  Vector3d pos_;
  Vector3d vel_;

  msg::BoatState* state_msg_;
  msg::Vector3f* wind_msg_;

  // The latest readings from the movable ballast encoder and inclinometer
  std::atomic<double> last_inclinometer_, last_ballast_;
  // The time at which we last received a ballast reading:
  util::monotonic_clock::time_point last_ballast_time_;
};

}  // namespace sailbot
}  // namespace control
