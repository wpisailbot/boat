#include "movable.h"

namespace sailbot {
namespace control {

Movable::Movable()
    : Node(dt), ballast_msg_(AllocateMessage<msg::BallastCmd>()),
      ballast_cmd_("ballast_cmd", true) {
  RegisterHandler<msg::BoatState>("boat_state",
                                  [this](const msg::BoatState &msg) {
    heel_ = msg.euler().roll();
  });

  RegisterHandler<msg::Vector3f>("wind", [this](const msg::Vector3f &msg) {
    if (std::atan2(msg.y(), msg.x()) > 0) {
      starboard_ = true;
    }
  });
}

void Movable::Iterate() {
  float goal_heel = starboard_ ? 0.5 : -0.5;
  ballast_msg_->set_pos(3.5 * (goal_heel-heel_));
  ballast_cmd_.send(ballast_msg_);
}

}  // control
}  // sailbot
