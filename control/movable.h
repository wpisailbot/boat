#pragma once
#include "util/node.h"

#include "control/actuator_cmd.pb.h"

namespace sailbot {
namespace control {
class Movable : public Node {
 public:
  Movable();

  void Iterate() override;

 private:
  constexpr static float dt = 0.01;

  std::atomic<double> heel_;
  std::atomic<bool> starboard_;

  msg::BallastCmd *ballast_msg_;
  ProtoQueue<msg::BallastCmd> ballast_cmd_;
};  // Movable

}  // namespace control
}  // namespace sailbot
