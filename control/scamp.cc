#include "scamp.h"

namespace sailbot {

namespace {

constexpr int kMinPot = 34;
constexpr int kMaxPot = kMinPot + 100 * 7;
float WinchPotToAngle(float pot_val) {
  constexpr int kPotRange = kMaxPot - kMinPot;
  return (1. - (pot_val - kMinPot) / kPotRange) * M_PI / 2;
}

}  // namespace

SCAMP::SCAMP()
    : Node(0.0), state_queue_("internal_state", true),
      state_msg_(AllocateMessage<msg::InternalBoatState>()),
      pwm_queue_("can65282", true),
      pwm_msg_(AllocateMessage<msg::can::CANMaster>()) {
  RegisterHandler<msg::can::CANMaster>("can65281" /*Analog Pot*/,
                                       [this](const msg::can::CANMaster &msg) {
    VLOG(2) << "Got pot val: " << msg.analog_pot().val();
    std::unique_lock<std::mutex> lck(state_msg_mut_);
    state_msg_->set_sail(
        WinchPotToAngle(msg.analog_pot().val())); // TODO(james): Add sign
    state_msg_->set_rudder((raw_rudder_ - 90.) * M_PI / 180.);
    state_queue_.send(state_msg_);
  });
  RegisterHandler<msg::SailCmd>("sail_cmd", [this](const msg::SailCmd &cmd) {
    float volts = cmd.voltage();
    // Don't allow it to run amok outside of bounds
    {
      std::unique_lock<std::mutex> lck(state_msg_mut_);
      float pos = state_msg_->sail();
      if (pos > M_PI / 2. - 0.1) {
        volts = std::min(volts, (float)0.);
      } else if (pos < 0.1) {
        volts = std::max(volts, (float)0.);
      }
    }
    int raw_val = volts / 12. * 90. + 90;

    raw_winch_ = raw_val;
  });
  RegisterHandler<msg::RudderCmd>("rudder_cmd", [this](const msg::RudderCmd &cmd) {
    int raw_val = cmd.pos() / M_PI * 180. + 90;

    raw_rudder_ = raw_val;
  });
}

void SCAMP::Iterate() {
  msg::can::PWMWrite *msg = pwm_msg_->mutable_pwm_write();
  msg->set_winch(raw_winch_);
  msg->set_rudder(raw_rudder_);
}

}  // namespace sailbot
