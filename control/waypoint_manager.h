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
    BACK,
    FORTH,
    LEAVE,
  };
  enum VisionStates {
    FOLLOW_WAYPOINTS,
    FOLLOW_VISION,
  };
  enum ObstacleState {
    WAYPOINT,
    AVOID,
  };

  void Iterate() override;
  void DoStationKeep();
  void DoVisionSearch();
  void DoObstacleAvoid();

  std::atomic<bool> tacker_done_{false};
  std::atomic<int> last_waypoint_{0};
  std::atomic<bool> last_sbus_manual_{false};
  std::atomic<bool> near_waypoint_{false};
  std::atomic<float> upwind_dir_{0};
  std::atomic<int> vision_confidence_{0};
  std::atomic<int> obs_cnt_{0};
  std::atomic<float> vision_abs_heading_{0};
  std::atomic<float> boat_yaw_{0};
  std::atomic<float> initial_avoid_diff_heading_{0};
  StationStates station_keep_state_{GOTO_START};
  VisionStates vision_state_{FOLLOW_WAYPOINTS};
  ObstacleState obstacle_state_{WAYPOINT};

  util::monotonic_clock::time_point station_keep_end_;

  msg::ChallengeControl::ChallengeType mode_{msg::ChallengeControl::WAYPOINT};

  std::mutex control_mode_mutex_;
  msg::ControlMode* control_mode_msg_;
  ProtoQueue<msg::ControlMode> control_mode_queue_;
  msg::RudderCmd* rudder_mode_msg_;
  ProtoQueue<msg::RudderCmd> rudder_mode_queue_;

  msg::SearchState* search_state_msg_;
  ProtoQueue<msg::SearchState> search_state_queue_;

  msg::HeadingCmd* heading_cmd_msg_;
  ProtoQueue<msg::HeadingCmd> heading_cmd_queue_;
}; // class WaypointManager

} // namespace control
} // namespace sailbot
