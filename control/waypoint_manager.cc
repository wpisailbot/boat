#include "control/waypoint_manager.h"

namespace sailbot {
namespace control {

WaypointManager::WaypointManager()
    : Node(0.1), station_keep_end_(Time()),
      control_mode_msg_(AllocateMessage<msg::ControlMode>()),
      control_mode_queue_("control_mode", true) {
  RegisterHandler<msg::SBUS>("sbus_value", [this](const msg::SBUS &msg) {
    if (msg.channel_size() >= 3) {
      std::unique_lock<std::mutex> l(control_mode_mutex_);
      control_mode_msg_->Clear();
      bool cur_sbus_manual = msg.channel(2) > 1000;
      if (cur_sbus_manual != last_sbus_manual) {
        if (cur_sbus_manual) {
          control_mode_msg_->set_rudder_mode(msg::ControlMode::FILTERED_RC);
          control_mode_msg_->set_winch_mode(msg::ControlMode::FILTERED_RC);
        } else {
          control_mode_msg_->set_rudder_mode(msg::ControlMode::AUTO);
          control_mode_msg_->set_winch_mode(msg::ControlMode::AUTO);
        }
      }
      last_sbus_manual_ = cur_sbus_manual;
      control_mode_queue_.send(control_mode_msg_);
    }
  });

  RegisterHandler<msg::TackerState>(
      "tacker_state",
      [this](const msg::TackerState &state) { tacker_done_ = state.done(); });
}

void WaypointManager::Iterate() {
  std::unique_lock<std::mutex> l(control_mode_mutex_);
  switch (mode_) {
    case msg::ChallengeControl::STATION_KEEP:
      if (tacker_done_) {
        // We've arrived at the waypoint in the middle of the station-keeping
        station_keep_end_ = Time() + std::chrono::milliseconds(1000);
      }
      break;
    case msg::ChallengeControl::WAYPOINT:
      control_mode_msg_->clear_tacker();
      break;
  }
}

} // namespace control
} // namespace sailbot
