#include "scamp.h"
#include "control/actuator_cmd.pb.h"

namespace sailbot {

namespace {

constexpr int kMinPot = 400;
constexpr int kMaxPot = 700;
float WinchPotToAngle(float pot_val) {
  constexpr int kPotRange = kMaxPot - kMinPot;
  return (1. - (pot_val - kMinPot) / kPotRange) * M_PI / 2;
}

}  // namespace

SCAMP::SCAMP()
    : Node(0.05), state_queue_("internal_state", true),
      state_msg_(AllocateMessage<msg::InternalBoatState>()),
      pwm_queue_("can65282", true),
      pwm_msg_(AllocateMessage<msg::can::CANMaster>()) {
  pwm_msg_->set_outgoing(true);
  RegisterHandler<msg::can::CANMaster>("can65281" /*Analog Pot*/,
                                       [this](const msg::can::CANMaster &msg) {
    VLOG(2) << "Got pot val: " << msg.analog_pot().val();
    std::unique_lock<std::mutex> lck(state_msg_mut_);
    state_msg_->set_sail(
        WinchPotToAngle(msg.analog_pot().val())); // TODO(james): Add sign
    state_msg_->set_rudder((raw_rudder_ - 90.) * -M_PI / 180.);
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
    // Also, create a deadband
    volts = std::abs(volts) < 1 ? 0 : volts;
    // And only permit 6V, until we know what we are doing:
    volts = std::min(std::max(volts, (float)-6.), (float)6.);
    int raw_val = volts / 12. * 90. + 90;

    if (is_connected_ && !is_manual_mode_) {
      raw_winch_ = raw_val;
    }
  });
  RegisterHandler<msg::RudderCmd>("rudder_cmd",
                                  [this](const msg::RudderCmd &cmd) {
    int raw_val = -cmd.pos() / M_PI * 180. + 90;
    VLOG(2) << "Got rudder command of " << raw_val;

    if (is_connected_ && !is_manual_mode_) {
      raw_rudder_ = raw_val;
    }
  });
  RegisterHandler<msg::SBUS>("sbus_value", [this](const msg::SBUS &sbus) {
    if ((is_manual_mode_ || !is_connected_) && sbus.channel_size() >= 2) {
      raw_rudder_ = SBUSToRaw(sbus.channel(1));
      raw_rudder_ = SBUSToRaw(sbus.channel(2));
    }
  });
  RegisterHandler<msg::ConnectionStatus>(
      "connection_status", [this](const msg::ConnectionStatus &status) {
    VLOG(2) << "Is connected? " << status.connected();
    is_connected_ = status.connected();
  });
  RegisterHandler<msg::ControlMode>(
      "control_mode", [this](const msg::ControlMode &mode) {
    is_manual_mode_ = mode.mode() == msg::ControlMode_MODE_MANUAL;
  });
}

void SCAMP::Iterate() {
  msg::can::PWMWrite *msg = pwm_msg_->mutable_pwm_write();
  VLOG(2) << "Sending message of " << msg->DebugString();
  msg->set_winch(raw_winch_);
  msg->set_rudder(raw_rudder_);
  pwm_queue_.send(pwm_msg_);
}

}  // namespace sailbot
