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
  enum StationStates {
    GOTO_START,
    WAIT,
    LEAVE,
  };
  enum VisionStates {
    FOLLOW_WAYPOINTS,
    FOLLOW_VISION,
  };

  void Iterate() override;
  void DoStationKeep();
  void DoVisionSearch();

  std::atomic<bool> tacker_done_{false};
  std::atomic<int> last_waypoint_{0};
  std::atomic<bool> last_sbus_manual_{false};
  std::atomic<float> upwind_dir_{0};
  std::atomic<int> vision_confidence_{0};
  std::atomic<float> vision_heading_{0};
  std::atomic<float> boat_yaw_{0};
  StationStates station_keep_state_{GOTO_START};
  VisionStates vision_state_{FOLLOW_WAYPOINTS};

  util::monotonic_clock::time_point station_keep_end_;

  msg::ChallengeControl::ChallengeType mode_{msg::ChallengeControl::WAYPOINT};

  std::mutex control_mode_mutex_;
  msg::ControlMode* control_mode_msg_;
  ProtoQueue<msg::ControlMode> control_mode_queue_;

  msg::HeadingCmd* heading_cmd_msg_;
  ProtoQueue<msg::HeadingCmd> heading_cmd_queue_;
}; // class WaypointManager

} // namespace control
} // namespace sailbot
