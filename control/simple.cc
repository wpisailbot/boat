#include "simple.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "sim/util.h"

namespace sailbot {
namespace control {

namespace {
  bool in_bounds(float alpha, float min, float max) {
    float diff = norm_angle(alpha - min);
    return diff > 0 && diff < norm_angle(max - min);
  }
}

SimpleControl::SimpleControl()
    : Node(0.01),
      sail_msg_(AllocateMessage<msg::SailCmd>()),
      rudder_msg_(AllocateMessage<msg::RudderCmd>()),
      boat_state_(AllocateMessage<msg::BoatState>()),
      sail_cmd_("sail_cmd", true),
      rudder_cmd_("rudder_cmd", true) {
  RegisterHandler<msg::BoatState>("boat_state", [this](const msg::BoatState &msg) {
    std::unique_lock<std::mutex> l(boat_state_mutex_);
    *boat_state_ = msg;
  });
  RegisterHandler<msg::Vector3f>("wind", [this](const msg::Vector3f &msg) {
    wind_x_ = msg.x();
    wind_y_ = msg.y();
  });
}

void SimpleControl::Iterate() {
  std::unique_lock<std::mutex> l(boat_state_mutex_);

  // Set angle of attack to be ~.4
  float vx = boat_state_->vel().x();
  float vy = boat_state_->vel().y();
  Eigen::Quaternionf orientation(
      boat_state_->orientation().w(), boat_state_->orientation().x(),
      boat_state_->orientation().y(), boat_state_->orientation().z());
  Eigen::Matrix3f rot = orientation.toRotationMatrix();
  float yaw = std::atan2(rot(1, 0), rot(0, 0));
  VLOG(2) << "yaw: " << yaw;

  // For alphaw, if sailing straight into wind, =0, if wind is coming from port
  // (ie, on port tack), alphaw is positive, if coming from starboard, is
  // negative/between pi and 2pi.
  float alphaw = norm_angle(std::atan2(vy - wind_y_, vx - wind_x_) - yaw);
  float wind_source_dir = std::atan2(-wind_y_, -wind_x_);
  float alphasail = alphaw - boat_state_->internal().sail();
  // If we are on port tack, want alphasail > 0, if on starboard, alphasail < 0.
  float goal = alphaw > 0 ? .4 : -.4;
  goal = std::abs(alphaw) < .4 ? 0 : goal;
  VLOG(2) << "Alphaw: " << alphaw << " alphas: " << alphasail
          << " goals: " << goal;
  sail_msg_->set_vel(-norm_angle(goal - alphasail));

  float goalx = 0, goaly = 100;
  float dx = goalx - boat_state_->pos().x();
  float dy = goaly - boat_state_->pos().y();
  float goal_heading = std::atan2(dy, dx);
  float min_upwind_angle = .7;
  float min_goal = norm_angle(wind_source_dir - min_upwind_angle);
  float max_goal = norm_angle(wind_source_dir + min_upwind_angle);
  bool lim_min = in_bounds(goal_heading, min_goal, wind_source_dir);
  bool lim_max = in_bounds(goal_heading, wind_source_dir, max_goal);
  VLOG(2) << "Wind src: " << wind_source_dir << " gh: " << goal_heading
          << " lim_min: " << lim_min << " lim_max: " << lim_max;
  if (lim_min || lim_max) {
    constexpr float kChange = 2.;
    constexpr float kFar = .1;
    constexpr float kHistCost = .01 * dt;
    float change_min = kChange * std::pow(std::abs(norm_angle(min_goal - last_goal_)) / M_PI, 3);
    float change_max = kChange * std::pow(std::abs(norm_angle(max_goal - last_goal_)) / M_PI, 3);
    float is_far_min = kFar * lim_max; // High numbers=penalty.
    float is_far_max = kFar * lim_min;
    float pen_min = change_min + is_far_min;
    float pen_max = change_max + is_far_max;
    float pen_hist_goal = kHistCost * goal_cost_;
    if (change_min < change_max) {
      pen_min += pen_hist_goal;
    } else {
      pen_max += pen_hist_goal;
    }
    VLOG(2) << "Tacking: ming " << min_goal << " maxg " << max_goal
            << " change_min " << change_min << " change_max " << change_max
            << " pen_hist " << pen_hist_goal
            << " pen_min " << pen_min << " pen_max " << pen_max << " goal "
            << goal_heading;
    goal_heading = (pen_min > pen_max) ? max_goal : min_goal;

    if ((is_far_min > is_far_max) != (change_min > change_max)) {
      goal_cost_ += 1;
    } else {
      goal_cost_ = 0;
    }
  } else {
    goal_cost_ = 0;
  }
  last_goal_ = goal_heading;

  float max_rudder = .4;
  //float boat_heading = std::atan2(vy, vx);
  float goal_rudder = std::min(
      std::max(-(goal_heading - yaw/*boat_heading*/), -max_rudder), max_rudder);
  VLOG(2) << "goalh: " << goal_heading << " goal_rudder: " << goal_rudder;
  rudder_msg_->set_vel(goal_rudder - boat_state_->internal().rudder());
  sail_cmd_.send(sail_msg_);
  rudder_cmd_.send(rudder_msg_);
}

}  // control
}  // sailbot
