#include "line_tacking.h"
#include "control/actuator_cmd.pb.h"
#include "util.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "gflags/gflags.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"

DEFINE_string(initial_waypoints, "waypoints.pba",
              "The initial set of waypoints to use when starting up");

namespace sailbot {
namespace control {

constexpr int LineTacker::N_WAYPOINTS;

LineTacker::LineTacker()
    : Node(0.1), kWindTol(0.1), kMinTackSpeed(1),
      cur_pos_({0, 0}), wind_dir_(0), cur_theta_(0),
      consts_msg_(AllocateMessage<msg::TackerConstants>()),
      consts_queue_("tacker_consts", true),
      heading_msg_(AllocateMessage<msg::HeadingCmd>()),
      heading_cmd_("heading_cmd", true),
      state_msg_(AllocateMessage<msg::TackerState>()),
      state_queue_("tacker_state", true) {

  ReadWaypointsFromFile(FLAGS_initial_waypoints.c_str());

  RegisterHandler<msg::TackerConstants>(
      "tacker_consts", [this](const msg::TackerConstants &msg) {
        std::unique_lock<std::mutex> lck(consts_mutex_);
        *consts_msg_ = msg;
      });

  consts_msg_->set_close_haul_angle(M_PI / 4);
  consts_msg_->set_in_irons_cost(3);
  consts_msg_->set_near_goal_cost(3);
  consts_msg_->set_hysteresis_cost(.3);
  consts_msg_->set_diff_yaw_cost(0);
  consts_msg_->set_momentum_cost(1);
  consts_msg_->set_tacking_cost(0);
  {
    std::unique_lock<std::mutex> lck(consts_mutex_);
    consts_queue_.send(consts_msg_);
  }

  RegisterHandler<msg::Vector3f>("true_wind", [this](const msg::Vector3f &msg) {
    wind_dir_ = std::atan2(msg.y(), msg.x());
  });
  RegisterHandler<msg::Vector3f>("wind",
                                 [this](const msg::Vector3f &msg) {
    apparent_wind_dir_ = std::atan2(msg.y(), msg.x());
  });
  // TODO(james): Thread-safeness.
  RegisterHandler<msg::BoatState>("boat_state",
                                  [this](const msg::BoatState &msg) {
    cur_theta_ = msg.euler().yaw();
    cur_speed_ = std::sqrt(msg.vel().x() * msg.vel().x() +
                           msg.vel().y() * msg.vel().y());
    cur_speed_ = 10;
    cur_pos_.x = msg.pos().x();
    cur_pos_.y = msg.pos().y();

    cur_yaw_rate_ = util::norm_angle(msg.omega().z());
  });

  RegisterHandler<msg::ControlMode>("control_mode",
                                    [this](const msg::ControlMode &msg) {
    if (msg.has_tacker()) {
      tack_mode_ = msg.tacker();
    }
  });

  RegisterHandler<msg::WaypointList>(
      "waypoints",
      std::bind(&LineTacker::ProcessWaypoints, this, std::placeholders::_1));
}

void LineTacker::ProcessWaypoints(const msg::WaypointList &msg) {
  // TODO(james): Thread-safety
  int starti = msg.restart() ? 0 : msg_i_offset_ + i_;
  recalc_zero_ = !msg.defined_start();
  int offset = recalc_zero_ ? 1 : 0;

  int cnt = std::min(msg.points_size() - starti, N_WAYPOINTS - offset);
  for (int i = 0; i < cnt; ++i) {
    waypoints_[i + offset] = {msg.points(i + starti).x(),
                              msg.points(i + starti).y()};
  }
  i_ = 0;
  msg_i_offset_ = starti;
  way_len_ = cnt + offset;
}

void LineTacker::ReadWaypointsFromFile(const char *fname) {
  if (fname == nullptr) {
    return;
  }
  int fd = open(fname, O_RDONLY);
  if (fd < 0) {
    PLOG(WARNING) << "Failed to open file " << fname;
    return;
  }
  google::protobuf::io::FileInputStream finput(fd);
  finput.SetCloseOnDelete(true);
  msg::WaypointList way_list;
  if (!google::protobuf::TextFormat::Parse(&finput, &way_list)) {
    LOG(WARNING) << "Failed to parse protobuf";
    return;
  }
  ProtoQueue<msg::WaypointList> q("waypoints", true);
  q.send(&way_list);
  ProcessWaypoints(way_list);
  LOG(INFO) << way_list.DebugString();
}

void LineTacker::Iterate() {
  if (way_len_ == 0) {
    LOG(INFO) << "No Waypoints--not doing anything";
    return;
  }
  if (tack_mode_ == msg::ControlMode::DISABLED) {
    LOG(INFO) << "Tacking disabled--not doing anything";
    return;
  }
  double gh = GoalHeading();
  heading_msg_->set_heading(gh);
  heading_cmd_.send(heading_msg_);

  std::unique_lock<std::mutex> lck(consts_mutex_);
  consts_queue_.send(consts_msg_);
}

// For all reward functions:
// Take a single heading, and return a value between 0 and 1,
// where 1 is good and 0 is bad. The values will then be scaled later.

// The reward for whether or not the heading will put us in irons.
float LineTacker::InIronsReward(float heading) {
  float upwind = util::norm_angle(wind_dir_ - M_PI);
  float abswdiff = std::abs(util::norm_angle(heading - upwind));
  float reward = std::min(abswdiff * 2 / M_PI, 1.);
  return reward;
}

// Whether or not a we really *want* to be pointed in a given
// direction. Basically, we want to be pointed straight towards the
// goal in an ideal world.
float LineTacker::DesirabilityReward(float heading, float nominal_heading) {
  float normdiff = util::norm_angle(heading - nominal_heading) / M_PI;
  return (1 - normdiff * normdiff);
}

// An estimate of how much momentum is working in our favor
// for the particular heading. We should consider
// both the forward speed and the angular momentum,
// but for now I just consider angular momentum.
float LineTacker::MomentumReward(float heading) {
  float diff = util::norm_angle(heading - cur_theta_) / M_PI;
  float diff2 = diff * diff;
  //float speed = -std::expm1(-cur_speed_);
  float norm_yaw_rate = cur_yaw_rate_ / M_PI * (diff > 0 ? 1 : -1);
  return (1-diff2) * norm_yaw_rate;
}

// A penalty for goal headings that are far away from
// our current heading. Goes hand in hand with hysteresis.
float LineTacker::CostToGoReward(float heading) {
  float diff = util::norm_angle(heading - cur_theta_) / M_PI;
  float diff2 = diff * diff;
  return 1 - diff2;
}

// Penalizes a heading for requiring that we spend time pointed upwind
// in order to go to it.
float LineTacker::RequiresTackingReward(float heading) {
  float diff = util::norm_angle(heading - cur_theta_);
  float wdiff = util::norm_angle(cur_theta_ - wind_dir_ + M_PI);
  float b = wdiff + diff;
  return std::abs(b - wdiff - std::sin(b) + std::sin(wdiff));
}

// Try to stay close to the most recently decided upon heading
float LineTacker::IndecisionReward(float heading) {
  float diff = util::norm_angle(heading - heading_msg_->heading()) / M_PI;
  float diff2 = diff * diff;
  return 1 - diff2;
}

/**
 * Basic algorithm:
 * If we have a straight-line path from our current position to the goal
 * position and the straight line isn't too close to being close-hauled, then
 * aim to go straight there.
 * Otherwise, depends on the state:
 *   -If not over/at the line boundary, go to whichever closehaul is closest.
 *   -If over/at the boundary, to closehaul that brings us into the line.
 */
float LineTacker::GoalHeading() {
  std::unique_lock<std::mutex> lck(consts_mutex_);
  if (recalc_zero_) {
    waypoints_[0] = cur_pos_;
  }
  Point start = waypoints_[i_];
  Point end = waypoints_[i_+1];
  float upwind = util::norm_angle(wind_dir_ - M_PI);
  float min_closehaul = util::norm_angle(upwind - consts_msg_->close_haul_angle());
  float max_closehaul = util::norm_angle(upwind + consts_msg_->close_haul_angle());
  float last_heading = heading_msg_->heading();
  float dy = end.y - cur_pos_.y;
  float dx = end.x - cur_pos_.x;
  float nominal_heading = std::atan2(dy, dx);
  float dist = std::sqrt(dy * dy + dx * dx);
  float goal_wind_diff = util::norm_angle(nominal_heading - upwind);

  bool done = false;
  if (dist < 5e-5) {
    done = true;
    if (i_ < way_len_ - 2) {
      ++i_;
      done = false;
    }
  }
  state_msg_->set_done(done);
  state_queue_.send(state_msg_);

  if (tack_mode_ == msg::ControlMode_TACKER_REWARD) {
    constexpr int NHEAD = 4;

    float possible_headings[NHEAD] = {nominal_heading, last_heading,
                                      min_closehaul, max_closehaul};

    float max_reward = 0;
    float best_heading = nominal_heading;

    for (int i = 0; i < NHEAD; ++i) {
      float h = possible_headings[i];
      float reward = consts_msg_->in_irons_cost() * InIronsReward(h) +
                     consts_msg_->near_goal_cost() *
                         DesirabilityReward(h, nominal_heading) +
                     consts_msg_->hysteresis_cost() * IndecisionReward(h) +
                     consts_msg_->diff_yaw_cost() * CostToGoReward(h) +
                     consts_msg_->momentum_cost() * MomentumReward(h) +
                     consts_msg_->tacking_cost() * RequiresTackingReward(h);
      if (reward > max_reward) {
        max_reward = reward;
        best_heading = h;
      }
    }

    return best_heading;
  } else if (tack_mode_ == msg::ControlMode_TACKER_LINE) {

    if (std::abs(goal_wind_diff) > consts_msg_->close_haul_angle() + kWindTol) {
      // It is worth it to go ahead and go straight there.
      VLOG(1) << "Going straight to " << nominal_heading << " i: " << i_;
      VLOG(1) << "endx: " << end.x << ", endy: " << end.y
              << ", startx: " << start.x << ", starty: " << end.y
              << ", curx: " << cur_pos_.x << ", cury: " << cur_pos_.y;
      return nominal_heading;
    }

    float dist_to_line = DistanceFromLine(start, end, cur_pos_) * 1e5;

    if (std::abs(dist_to_line) >= bounds_[i_]) {
      // Too far from the path, so go back, but only if we will have
      // enough speed to tack.
      VLOG(1) << "Outside bounds and moving fast";
      if (util::norm_angle(wind_dir_ - nominal_heading) > 0) {
        return max_closehaul;
      } else {
        return min_closehaul;
      }
    } else {
      // Close enough to the line or too slow, so go wherever is closest.
      //if (apparent_wind_dir_ < 0) {
      VLOG(1) << "Inside bounds or moving slow ap wind: " << ApparentWind()
              << " yaw: " << cur_theta_ << " wind: " << wind_dir_
              << " app wind dir: " << apparent_wind_dir_;
      if (std::abs(ApparentWind()) < M_PI / 2) {
        VLOG(1) << "Going downwind";
        if (ApparentWind() < 0) {
          return min_closehaul;
        } else {
          return max_closehaul;
        }
      } else {
        VLOG(1) << "Going upwind-ish";
        float diff_min_close =
            std::abs(util::norm_angle(min_closehaul - cur_theta_));
        float diff_max_close =
            std::abs(util::norm_angle(max_closehaul - cur_theta_));
        if (diff_min_close < diff_max_close) {
          return min_closehaul;
        } else {
          return max_closehaul;
        }
      }
    }
  } else {
    return nominal_heading;
  }

  LOG(FATAL) << "Reached unreachable state";
}

float LineTacker::DistanceFromLine(Point start, Point end, Point loc) {
  // Account for instances when we are near singularities:
  if (std::abs(end.y - start.y) < 1e-3) {
    return (start.y + end.y) / 2 - loc.y;
  } else if (std::abs(end.x - start.x) < 1e-3) {
    return (start.x + end.x) / 2 - loc.x;
  }
  // Draw a line that goes through loc and is perpendicular to the original.
  // Express it as y = perp_slope * x + perp_intercept;
  float perp_slope = -(end.x - start.x) / (end.y - start.y);
  float perp_intercept = loc.y - perp_slope * loc.x;
  float orig_slope = -1 / perp_slope;
  float orig_intercept = end.y - orig_slope * end.x;
  // Now, find intersection between perp_line and original line.
  float x = (perp_intercept - orig_intercept) / (orig_slope - perp_slope);
  float y = perp_slope * x + perp_intercept;
  float dx = x - loc.x;
  float dy = y - loc.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  dist *= dx > 0 ? 1 : -1;
  return dist;
}

float LineTacker::ApparentWind() {
  //return util::norm_angle(wind_dir_ - cur_theta_);
  return util::norm_angle(apparent_wind_dir_);
}

}  // namespace control
}  // namespace sailbot
