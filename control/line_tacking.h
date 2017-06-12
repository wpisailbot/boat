#pragma once

#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include <mutex>

namespace sailbot {
namespace control {

class LineTacker : public Node {
 public:
  struct Point {
    double x;
    double y;
  };

  LineTacker();
 private:
  void Iterate() override;
  const float kWindTol; // Amount of extra margin to give before tacking (small positive num)
  const float kMinTackSpeed; // Minimum speed required to execute a tack
  static constexpr int N_WAYPOINTS = 10;

  float GoalHeading();

  void ProcessWaypoints(const msg::WaypointList &msg);
  void ReadWaypointsFromFile(const char *fname);

  // Distance of loc from the line between start and end
  float DistanceFromLine(Point start, Point end, Point loc);
  float ApparentWind(); // Direction wind is coming FROM

  // Utility reward functions
  float InIronsReward(float heading);
  float DesirabilityReward(float heading, float nominal_heading);
  float MomentumReward(float heading);
  float RequiresTackingReward(float heading);
  float IndecisionReward(float heading);
  float CostToGoReward(float heading);

  Point waypoints_[N_WAYPOINTS];
  float bounds_[N_WAYPOINTS] = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20};
  int i_ = 0;
  int msg_i_offset_ = 0;
  int way_len_ = 0;
  std::atomic<bool> recalc_zero_{false};

  Point cur_pos_;
  std::atomic<float> wind_dir_; // The direction the wind is blowing TO
  std::atomic<float> apparent_wind_dir_;
  std::atomic<float> cur_theta_; // The current boat heading
  std::atomic<float> cur_speed_; // m/s
  std::atomic<float> cur_yaw_rate_; // rad/s

  std::atomic<int> tack_mode_{msg::ControlMode_TACKER_REWARD};

  std::mutex consts_mutex_;
  msg::TackerConstants *consts_msg_;
  ProtoQueue<msg::TackerConstants> consts_queue_;

  msg::HeadingCmd* heading_msg_;
  ProtoQueue<msg::HeadingCmd> heading_cmd_;

  msg::TackerState* state_msg_;
  ProtoQueue<msg::TackerState> state_queue_;
};

}  // namespace control
}  // namespace sailbot
