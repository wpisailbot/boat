#pragma once

#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include <mutex>

namespace sailbot {
namespace control {

class WaypointManager : public Node {
 public:
  WaypointManager();
 private:
  void Iterate() override;

  std::atomic<bool> tacker_done_{false};
  std::atomic<bool> last_sbus_manual_{false};

  util::monotonic_clock::time_point station_keep_end_;

  msg::ChallengeControl::ChallengeType mode_{msg::ChallengeControl::WAYPOINT};

  std::mutex control_mode_mutex_;
  msg::ControlMode* control_mode_msg_;
  ProtoQueue<msg::ControlMode> control_mode_queue_;
}; // class WaypointManager

} // namespace control
} // namespace sailbot
