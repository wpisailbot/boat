#include "scamp.h"
#include "control/actuator_cmd.pb.h"
#include "gflags.h"

namespace sailbot {

SCAMP::SCAMP()
    : Node(0.05), state_queue_("internal_state", true),
      state_msg_(AllocateMessage<msg::InternalBoatState>()),
      pwm_queue_("can65282", true),
      pwm_msg_(AllocateMessage<msg::can::CANMaster>()),
      consts_queue_("zeroing_consts", true),
      consts_msg_(AllocateMessage<msg::ZeroingConstants>()) {
  pwm_msg_->set_outgoing(true);

  consts_msg_->set_rudder_zero(99);
  consts_msg_->set_winch_0_pot(985);
  consts_msg_->set_winch_90_pot(685);

  RegisterHandler<msg::ZeroingConstants>(
      "zeroing_consts", [this](const msg::ZeroingConstants &msg) {
        if (msg.has_winch_0_pot() && msg.has_winch_90_pot()) {
          consts_msg_->set_winch_0_pot(msg.winch_0_pot());
          consts_msg_->set_winch_90_pot(msg.winch_90_pot());
        }
        if (msg.has_rudder_zero()) {
          consts_msg_->set_rudder_zero(msg.rudder_zero());
        }
      });

  RegisterHandler<msg::can::CANMaster>("can65281" /*Analog Pot*/,
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mut_);
    state_msg_->set_sail(
        WinchPotToAngle(msg.analog_pot().val())); // TODO(james): Add sign
    state_msg_->set_rudder((raw_rudder_ - consts_msg_->rudder_zero()) * -M_PI / 180.);
    state_queue_.send(state_msg_);
  });
  RegisterHandler<msg::SailCmd>("sail_cmd", [this](const msg::SailCmd &cmd) {
    if (IsAuto(WINCH)) {
      volts_winch_ = cmd.voltage();
    }
  });

  RegisterHandler<msg::SailCmd>("manual_sail_cmd",
                                [this](const msg::SailCmd &cmd) {
    if (IsManualWiFi(WINCH)) {
      volts_winch_ = is_connected_ ? cmd.voltage() : 0.;
    }
  });

  RegisterHandler<msg::RudderCmd>("rudder_cmd",
                                  [this](const msg::RudderCmd &cmd) {
    if (IsAuto(RUDDER)) {
      SetRawFromRudderCmd(cmd);
    }
  });

  RegisterHandler<msg::RudderCmd>("manual_rudder_cmd",
                                  [this](const msg::RudderCmd &cmd) {
    if (IsManualWiFi(RUDDER)) {
      SetRawFromRudderCmd(cmd);
    }
  });

  RegisterHandler<msg::SBUS>("sbus_value", [this](const msg::SBUS &sbus) {
    if (sbus.channel_size() >= 2) {
      if (IsManualRC(RUDDER)) {
        raw_rudder_ = (SBUSToRaw(sbus.channel(0)) - 90.) * .5 + 90;
      } else if (IsFilteredRC(RUDDER)) {
        raw_rudder_ =
            (SBUSToRaw(sbus.channel(0)) - 90.) * .3 + consts_msg_->rudder_zero();
      }

      if (IsManualRC(WINCH)) {
        raw_winch_ = SBUSToRaw(sbus.channel(1));
      } else if (IsFilteredRC(WINCH)) {
        volts_winch_ = (SBUSToRaw(sbus.channel(1)) - 90.) * 6. / 90.;
      }
    }
  });
  RegisterHandler<msg::ConnectionStatus>(
      "connection_status", [this](const msg::ConnectionStatus &status) {
    is_connected_ = status.connected();
  });
  RegisterHandler<msg::ControlMode>(
      "control_mode", [this](const msg::ControlMode &mode) {
    if (mode.has_winch_mode()) {
      winch_mode_ = mode.winch_mode();
    }
    if (mode.has_rudder_mode()) {
      rudder_mode_ = mode.rudder_mode();
    }
    if (IsDisabled(RUDDER)) {
      raw_rudder_ = 90;
    }
    if (IsDisabled(WINCH)) {
      raw_winch_ = 90;
    }
  });
}

void SCAMP::Iterate() {
  msg::can::PWMWrite *msg = pwm_msg_->mutable_pwm_write();
  VLOG(2) << "Sending message of " << msg->DebugString();
  if (IsAuto(WINCH) || IsManualWiFi(WINCH) || IsFilteredRC(WINCH)) {
    SetRawFromSailCmd(volts_winch_);
  }
  msg->set_winch(raw_winch_);
  msg->set_rudder(raw_rudder_);
  pwm_queue_.send(pwm_msg_);
  consts_queue_.send(consts_msg_);
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
  volts *= (consts_msg_->winch_90_pot() > consts_msg_->winch_0_pot()) ? -1 : 1;
  int raw_val = volts / 12. * 90. + 90;

  raw_winch_ = raw_val;
}

void SCAMP::SetRawFromRudderCmd(const msg::RudderCmd &cmd) {
  int raw_val = -cmd.pos() / M_PI * 180. + consts_msg_->rudder_zero();

  raw_rudder_ = raw_val;
}

float SCAMP::WinchPotToAngle(float pot_val) {
  const int kPotRange =
      consts_msg_->winch_90_pot() - consts_msg_->winch_0_pot();
  return (pot_val - consts_msg_->winch_0_pot()) / kPotRange * M_PI / 2;
}

}  // namespace sailbot
