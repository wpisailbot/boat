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
  const float kCloseHaul; // How close we can sail to the wind (positive num, ~PI/4)
  const float kWindTol; // Amount of extra margin to give before tacking (small positive num)
  static constexpr int N_WAYPOINTS = 10;

  float GoalHeading();

  // Distance of loc from the line between start and end
  float DistanceFromLine(Point start, Point end, Point loc);
  float ApparentWind();

  Point waypoints_[N_WAYPOINTS];
  float bounds_[N_WAYPOINTS] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
  int i_ = 0;

  std::atomic<float> wind_dir_;
  Point cur_pos_;
  std::atomic<float> cur_theta_;

  msg::HeadingCmd* heading_msg_;
  ProtoQueue<msg::HeadingCmd> heading_cmd_;
};

}  // namespace control
}  // namespace sailbot
