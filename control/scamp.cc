#include "scamp.h"
#include "control/actuator_cmd.pb.h"
#include "gflags.h"

DEFINE_int32(full_out_pot, 500, "Pot value at winch = 90 deg");
DEFINE_int32(full_in_pot, 800, "Pot value at winch = 0 deg");

namespace sailbot {

namespace {

float WinchPotToAngle(float pot_val) {
  const int kPotRange = FLAGS_full_out_pot - FLAGS_full_in_pot;
  return (pot_val - FLAGS_full_in_pot) / kPotRange * M_PI / 2;
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
    std::unique_lock<std::mutex> lck(state_msg_mut_);
    state_msg_->set_sail(
        WinchPotToAngle(msg.analog_pot().val())); // TODO(james): Add sign
    state_msg_->set_rudder((raw_rudder_ - 90.) * -M_PI / 180.);
    state_queue_.send(state_msg_);
  });
  RegisterHandler<msg::SailCmd>("sail_cmd", [this](const msg::SailCmd &cmd) {
    if (IsAuto()) {
      SetRawFromSailCmd(cmd.voltage());
    }
  });

  RegisterHandler<msg::SailCmd>("manual_sail_cmd",
                                [this](const msg::SailCmd &cmd) {
    if (IsManualWiFi()) {
      SetRawFromSailCmd(is_connected_ ? cmd.voltage() : 0.);
    }
  });

  RegisterHandler<msg::RudderCmd>("rudder_cmd",
                                  [this](const msg::RudderCmd &cmd) {
    if (IsAuto()) {
      SetRawFromRudderCmd(cmd);
    }
  });

  RegisterHandler<msg::RudderCmd>("manual_rudder_cmd",
                                  [this](const msg::RudderCmd &cmd) {
    if (IsManualWiFi()) {
      SetRawFromRudderCmd(cmd);
    }
  });

  RegisterHandler<msg::SBUS>("sbus_value", [this](const msg::SBUS &sbus) {
    if (IsManualRC() && sbus.channel_size() >= 2) {
      raw_rudder_ = (SBUSToRaw(sbus.channel(0)) - 90.) * .5 + 90;
      raw_winch_ = SBUSToRaw(sbus.channel(1));
    }
  });
  RegisterHandler<msg::ConnectionStatus>(
      "connection_status", [this](const msg::ConnectionStatus &status) {
    is_connected_ = status.connected();
  });
  RegisterHandler<msg::ControlMode>(
      "control_mode", [this](const msg::ControlMode &mode) {
    control_mode_ = mode.mode();
    if (IsDisabled()) {
      raw_winch_ = 90;
      raw_rudder_ = 90;
    }
  });
}

void SCAMP::Iterate() {
  msg::can::PWMWrite *msg = pwm_msg_->mutable_pwm_write();
  VLOG(2) << "Sending message of " << msg->DebugString();
  msg->set_winch(raw_winch_);
  msg->set_rudder(raw_rudder_);
  pwm_queue_.send(pwm_msg_);
}

void SCAMP::SetRawFromSailCmd(float volts) {
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
  volts *= (FLAGS_full_out_pot > FLAGS_full_in_pot) ? -1 : 1;
  int raw_val = volts / 12. * 90. + 90;

  raw_winch_ = raw_val;
}

void SCAMP::SetRawFromRudderCmd(const msg::RudderCmd &cmd) {
  int raw_val = -cmd.pos() / M_PI * 180. + 90;

  raw_rudder_ = raw_val;
}

}  // namespace sailbot
