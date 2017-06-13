#include "control/waypoint_manager.h"
#include "control/util.h"

namespace sailbot {
namespace control {

static constexpr int kStationKeepSeconds = 180;
static constexpr int kVisionConfidenceThreshold = 2;
static constexpr int kVisionFoundThreshold = 30;

WaypointManager::WaypointManager()
    : Node(0.1), station_keep_end_(Time()),
      control_mode_msg_(AllocateMessage<msg::ControlMode>()),
      control_mode_queue_("control_mode", true),
      heading_cmd_msg_(AllocateMessage<msg::HeadingCmd>()),
      heading_cmd_queue_("heading_cmd", true) {
  RegisterHandler<msg::SBUS>("sbus_value", [this](const msg::SBUS &msg) {
    if (msg.channel_size() >= 3) {
      std::unique_lock<std::mutex> l(control_mode_mutex_);
      control_mode_msg_->Clear();
      bool cur_sbus_manual = msg.channel(2) < 1000;
      if ((msg.channel(2) < 800 || msg.channel(2) > 1200) && cur_sbus_manual != last_sbus_manual_) {
        if (cur_sbus_manual) {
          control_mode_msg_->set_rudder_mode(msg::ControlMode::FILTERED_RC);
          control_mode_msg_->set_winch_mode(msg::ControlMode::FILTERED_RC);
        } else {
          control_mode_msg_->set_rudder_mode(msg::ControlMode::AUTO);
          control_mode_msg_->set_winch_mode(msg::ControlMode::AUTO);
        }
        control_mode_queue_.send(control_mode_msg_);
        last_sbus_manual_ = cur_sbus_manual;
      }
    }
  });

  RegisterHandler<msg::TackerState>("tacker_state",
                                    [this](const msg::TackerState &state) {
    tacker_done_ = state.done();
    last_waypoint_ = state.last_waypoint();
  });

  RegisterHandler<msg::ChallengeControl>(
      "challenge_control", [this](const msg::ChallengeControl &msg) {
        if (!msg.has_challenge()) {
          return;
        }
        mode_ = msg.challenge();
        switch (msg.challenge()) {
        case msg::ChallengeControl::STATION_KEEP:
          station_keep_state_ = GOTO_START;
          control_mode_msg_->set_rudder_mode(msg::ControlMode::AUTO);
          control_mode_msg_->set_winch_mode(msg::ControlMode::AUTO);
          control_mode_msg_->set_tacker(msg::ControlMode::REWARD);
          control_mode_queue_.send(control_mode_msg_);
          break;
        case msg::ChallengeControl::WAYPOINT:
          break;
        case msg::ChallengeControl::VISION:
          vision_state_ = FOLLOW_WAYPOINTS;
          control_mode_msg_->set_rudder_mode(msg::ControlMode::AUTO);
          control_mode_msg_->set_winch_mode(msg::ControlMode::AUTO);
          control_mode_msg_->set_tacker(msg::ControlMode::REWARD);
          control_mode_queue_.send(control_mode_msg_);
          break;
        }
      });

  RegisterHandler<msg::Vector3f>("true_wind", [this](const msg::Vector3f &msg) {
    upwind_dir_ = std::atan2(-msg.y(), -msg.x());
  });
  RegisterHandler<msg::VisionData>("vision_data",
                                   [this](const msg::VisionData &data) {
    if (data.has_heading() && data.has_confidence()) {
      vision_heading_ = data.heading();
      vision_confidence_ = data.confidence();
    }
  });
  RegisterHandler<msg::BoatState>("boat_state", [this](const msg::BoatState &msg) {
    boat_yaw_ = msg.euler().yaw();
  });
}

void WaypointManager::Iterate() {
  switch (mode_) {
    case msg::ChallengeControl::STATION_KEEP:
      DoStationKeep();
      break;
    case msg::ChallengeControl::WAYPOINT:
      break;
    case msg::ChallengeControl::VISION:
      DoVisionSearch();
      break;
  }
}

void WaypointManager::DoStationKeep() {
  std::unique_lock<std::mutex> l(control_mode_mutex_);
  switch (station_keep_state_) {
    case GOTO_START:
      if (tacker_done_) {
        station_keep_end_ = Time() + std::chrono::seconds(kStationKeepSeconds);
        station_keep_state_ = WAIT;
        control_mode_msg_->Clear();
        control_mode_msg_->set_tacker(msg::ControlMode::DISABLED);
        control_mode_queue_.send(control_mode_msg_);
      }
      break;
    case WAIT:
      heading_cmd_msg_->set_heading(upwind_dir_ + M_PI / 4);
      heading_cmd_msg_->set_extra_sail(0.5);
      heading_cmd_queue_.send(heading_cmd_msg_);
      heading_cmd_msg_->Clear();
      if (Time() > station_keep_end_) {
        station_keep_state_ = LEAVE;
      }
      break;
    case LEAVE:
      heading_cmd_msg_->set_heading(util::norm_angle(upwind_dir_ + M_PI * .75));
      heading_cmd_queue_.send(heading_cmd_msg_);
      break;
  }
}

void WaypointManager::DoVisionSearch() {
  float true_heading = vision_heading_ + boat_yaw_;
  switch (vision_state_) {
    case FOLLOW_WAYPOINTS:
      if ((tacker_done_ || last_waypoint_ >= 1) &&
          vision_confidence_ > kVisionConfidenceThreshold) {
        control_mode_msg_->Clear();
        control_mode_msg_->set_tacker(msg::ControlMode::DISABLED);
        control_mode_queue_.send(control_mode_msg_);
        vision_state_ = FOLLOW_VISION;
      }
      break;
    case FOLLOW_VISION:
      heading_cmd_msg_->set_heading(true_heading);
      heading_cmd_queue_.send(heading_cmd_msg_);
      if (vision_confidence_ > kVisionFoundThreshold) {
        control_mode_msg_->set_tacker(msg::ControlMode::DISABLED);
        control_mode_msg_->set_winch_mode(msg::ControlMode::DISABLE);
        control_mode_msg_->set_rudder_mode(msg::ControlMode::DISABLE);
        control_mode_queue_.send(control_mode_msg_);
      } else if (vision_confidence_ < kVisionConfidenceThreshold) {
        control_mode_msg_->Clear();
        control_mode_msg_->set_tacker(msg::ControlMode::REWARD);
        control_mode_queue_.send(control_mode_msg_);
      }
      break;
  }
}

} // namespace control
} // namespace sailbot
