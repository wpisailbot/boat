#include "scamp.h"
#include "control/actuator_cmd.pb.h"
#include "control/util.h"
#include "util/proto_util.h"
#include "gflags.h"

DEFINE_string(consts_file, "zero_consts.pba",
              "The file containing the intiial constants for zeroing");

namespace sailbot {

SCAMP::SCAMP()
    : Node(0.05), state_queue_("internal_state", true),
      state_msg_(AllocateMessage<msg::InternalBoatState>()),
      pwm_queue_("can65282", true),
      pwm_msg_(AllocateMessage<msg::can::CANMaster>()),
      consts_queue_("zeroing_consts", true),
      consts_msg_(AllocateMessage<msg::ZeroingConstants>()) {
  pwm_msg_->set_outgoing(true);

  if (!util::ReadProtoFromFile(FLAGS_consts_file.c_str(), consts_msg_)) {
    consts_msg_->set_rudder_zero(99);
    consts_msg_->set_ballast_zero(-1.775);
    consts_msg_->set_winch_0_pot(0);
    consts_msg_->set_winch_90_pot(1023);
  }

  RegisterHandler<msg::ZeroingConstants>(
      "zeroing_consts", [this](const msg::ZeroingConstants &msg) {
        if (msg.has_winch_0_pot() && msg.has_winch_90_pot()) {
          consts_msg_->set_winch_0_pot(msg.winch_0_pot());
          consts_msg_->set_winch_90_pot(msg.winch_90_pot());
        }
        if (msg.has_rudder_zero()) {
          consts_msg_->set_rudder_zero(msg.rudder_zero());
        }
        if (msg.has_ballast_zero()) {
          consts_msg_->set_ballast_zero(msg.ballast_zero());
        }
        if (msg.write_constants()) {
          util::WriteProtoToFile(FLAGS_consts_file.c_str(), *consts_msg_);
        }
      });

  RegisterHandler<msg::can::CANMaster>("can65281" /*Analog Pot*/,
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mut_);
    state_msg_->Clear();
    sail_pos_ = WinchPotToAngle(msg.analog_pot().val());
    state_msg_->set_sail(sail_pos_); // TODO(james): Add sign
    state_msg_->set_rudder((raw_rudder_ - consts_msg_->rudder_zero()) * M_PI / 180.);
    state_queue_.send(state_msg_);
  });

  RegisterHandler<msg::can::CANMaster>("can65285" /*Ballast State*/,
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mut_);
    if (msg.has_ballast_state() && msg.ballast_state().has_ballast()) {
      state_msg_->Clear();
      state_msg_->set_ballast(msg.ballast_state().ballast() -
                              consts_msg_->ballast_zero());
      state_queue_.send(state_msg_);
    }
  });

  RegisterHandler<msg::SailCmd>("sail_cmd", [this](const msg::SailCmd &cmd) {
    if (IsAuto(WINCH)) {
      volts_winch_ = cmd.voltage();
    }
  });

  RegisterHandler<msg::SailCmd>("manual_sail_cmd",
                                [this](const msg::SailCmd &cmd) {
    if (IsManualWiFi(WINCH)) {
      volts_winch_ = cmd.voltage();
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

  RegisterHandler<msg::BallastCmd>("ballast_cmd",
                                   [this](const msg::BallastCmd &cmd) {
    if (IsAuto(BALLAST)) {
      SetRawFromBallastCmd(cmd);
    }
  });

  RegisterHandler<msg::BallastCmd>("manual_ballast_cmd",
                                  [this](const msg::BallastCmd &cmd) {
    if (IsManualWiFi(BALLAST)) {
      SetRawFromBallastCmd(cmd);
    }
  });

  RegisterHandler<msg::SBUS>("sbus_value", [this](const msg::SBUS &sbus) {
    if (sbus.channel_size() >= 2) {
      if (IsManualRC(RUDDER)) {
        raw_rudder_ = (SBUSToRaw(sbus.channel(3)) - 90.) * .5 + 90;
      } else if (IsFilteredRC(RUDDER)) {
        raw_rudder_ =
            -(SBUSToRaw(sbus.channel(3)) - 90.) * .6 + consts_msg_->rudder_zero();
      }

      if (IsManualRC(WINCH)) {
        raw_winch_ = SBUSToRaw(sbus.channel(1));
      } else if (IsFilteredRC(WINCH)) {
        volts_winch_ = -(SBUSToRaw(sbus.channel(1)) - 90.) * 6. / 90.;
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
    if (mode.has_ballast_mode()) {
      ballast_mode_ = mode.ballast_mode();
    }
    if (IsDisabled(RUDDER)) {
      raw_rudder_ = 90;
    }
    if (IsDisabled(WINCH)) {
      raw_winch_ = 90;
    }
    if (IsDisabled(BALLAST)) {
      raw_ballast_ = 90;
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
  msg->set_ballast(raw_ballast_);
  pwm_queue_.send(pwm_msg_);
  consts_queue_.send(consts_msg_);
}

void SCAMP::SetRawFromSailCmd(float volts) {
  // Don't allow it to run amok outside of bounds
  {
    std::unique_lock<std::mutex> lck(state_msg_mut_);
    if (sail_pos_ > M_PI / 2. - 0.1) {
      volts = std::min(volts, (float)0.);
    } else if (sail_pos_ < 0.1) {
      volts = std::max(volts, (float)0.);
    }
  }
  // Also, create a deadband
  volts = std::abs(volts) < 1 ? 0 : volts;
  // And only permit 6V, until we know what we are doing:
  volts = std::min(std::max(volts, (float)-12.), (float)12.);
  volts *= (consts_msg_->winch_90_pot() > consts_msg_->winch_0_pot()) ? -1 : 1;
  int raw_val = volts / 12. * 90. + 90;

  raw_winch_ = raw_val;
}

void SCAMP::SetRawFromRudderCmd(const msg::RudderCmd &cmd) {
  int raw_val = cmd.pos() / M_PI * 180. + consts_msg_->rudder_zero();

  raw_rudder_ = raw_val;
}

void SCAMP::SetRawFromBallastCmd(const msg::BallastCmd &cmd) {
  if (cmd.has_vel()) {
    raw_ballast_ = cmd.vel() + 90;
  } else if (cmd.has_voltage()) {
    double volts = util::Clip((double)cmd.voltage(), -12.0, 12.0);
    raw_ballast_ = volts * 90.0 / 12.0 + 90.0;
  }
}

float SCAMP::WinchPotToAngle(float pot_val) {
  const int kPotRange =
      consts_msg_->winch_90_pot() - consts_msg_->winch_0_pot();
  return (pot_val - consts_msg_->winch_0_pot()) / kPotRange * M_PI / 2;
}

}  // namespace sailbot
