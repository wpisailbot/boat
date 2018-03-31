#include "monitor.h"

namespace sailbot {

Monitor::Monitor()
    : Node(1), state_msg_(AllocateMessage<msg::can::CANMaster>()),
      connection_msg_(AllocateMessage<msg::can::CANMaster>()),
      state_queue_("can65296", true), connection_queue_("can65297", true) {
  state_msg_->mutable_sailbot_state()->set_rigid(msg::ControlMode_MODE_DISABLE);
  state_msg_->mutable_sailbot_state()->set_ballast(
      msg::ControlMode_MODE_DISABLE);
  state_msg_->mutable_sailbot_state()->set_winch(msg::ControlMode_MODE_DISABLE);
  state_msg_->mutable_sailbot_state()->set_rudder(
      msg::ControlMode_MODE_DISABLE);
  state_msg_->mutable_sailbot_state()->set_tacker(msg::ControlMode_TACKER_NONE);
  RegisterHandler<msg::ConnectionStatus>(
      "connection_status", [this](const msg::ConnectionStatus &status) {
    ui_connection_ = status.connected();
  });
  RegisterHandler<msg::ControlMode>("control_mode",
                                    [this](const msg::ControlMode &mode) {
    std::unique_lock<std::mutex> l(state_mutex_);
    msg::can::SailbotState *state = state_msg_->mutable_sailbot_state();
    if (mode.has_rigid_mode()) {
      state->set_rigid(mode.rigid_mode());
    }
    if (mode.has_winch_mode()) {
      state->set_winch(mode.winch_mode());
    }
    if (mode.has_rudder_mode()) {
      state->set_rudder(mode.rudder_mode());
    }
    if (mode.has_ballast_mode()) {
      state->set_ballast(mode.ballast_mode());
    }
    if (mode.has_tacker()) {
      state->set_tacker(mode.tacker());
    }
  });
  RegisterHandler<msg::RigidWingFeedback>(
      "rigid_wing_feedback", [this](const msg::RigidWingFeedback &) {
        std::unique_lock<std::mutex> l(last_rigid_conn_mutex_);
        last_rigid_conn_ = Time();
        rigid_connection_ = true;
      });
}

void Monitor::Iterate() {
  {
    std::unique_lock<std::mutex> l(last_rigid_conn_mutex_);
    if (Time() > last_rigid_conn_ + std::chrono::seconds(kRigidTimeout)) {
      rigid_connection_ = false;
    }
  }

  state_msg_->set_outgoing(true);

  std::unique_lock<std::mutex> l(state_mutex_);

  msg::can::SailbotConn *conn = connection_msg_->mutable_sailbot_conn();

  conn->set_rigid_conn(rigid_connection_
                           ? msg::can::SailbotConn_ConnectionState_GOOD
                           : msg::can::SailbotConn_ConnectionState_BAD);
  conn->set_ui_conn(ui_connection_
                        ? msg::can::SailbotConn_ConnectionState_GOOD
                        : msg::can::SailbotConn_ConnectionState_BAD);

  conn->set_wifi_conn(msg::can::SailbotConn_ConnectionState_UNKNOWN);
  conn->set_radio_conn(msg::can::SailbotConn_ConnectionState_UNKNOWN);
  conn->set_jetson_conn(msg::can::SailbotConn_ConnectionState_UNKNOWN);

  state_queue_.send(state_msg_);
  connection_queue_.send(connection_msg_);
}

}  // namespace sailbot
