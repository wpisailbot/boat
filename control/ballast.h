#pragma once
#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include "control/util.h"
#include <mutex>

namespace sailbot {
namespace control {

class BallastControl : public Node {
 public:
  BallastControl();

  void Iterate() override;
 private:
  constexpr static float dt = 0.01;

  msg::BallastCmd *ballast_msg_;
  msg::ControllerConstants *consts_msg_;
  std::mutex consts_mutex_;
  ProtoQueue<msg::BallastCmd> ballast_cmd_;
  ProtoQueue<msg::ControllerConstants> consts_queue_;

  std::atomic<double> heel_goal_{0};
  std::atomic<double> heel_{0};
  std::atomic<double> heel_dot_{0};
  std::atomic<double> ballast_{0};
  std::atomic<double> ballast_dot_{0};

  std::atomic<double> heel_error_integrator_{0};

  int counter_ = 0;
};

}  // control
}  // sailbot
