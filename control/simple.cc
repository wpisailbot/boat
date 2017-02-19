#include "simple.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "control/util.h"

namespace sailbot {
namespace control {

namespace {
  bool in_bounds(float alpha, float min, float max) {
    float diff = util::norm_angle(alpha - min);
    return diff > 0 && diff < util::norm_angle(max - min);
  }
}

SimpleControl::SimpleControl(bool do_rudder)
    : Node(0.01),
      do_rudder_(do_rudder),
      sail_msg_(AllocateMessage<msg::SailCmd>()),
      rudder_msg_(AllocateMessage<msg::RudderCmd>()),
      ballast_msg_(AllocateMessage<msg::BallastCmd>()),
      boat_state_(AllocateMessage<msg::BoatState>()),
      heading_(0),
      sail_cmd_("sail_cmd", true),
      rudder_cmd_("rudder_cmd", true),
      ballast_cmd_("ballast_cmd", true) {
  RegisterHandler<msg::BoatState>("boat_state", [this](const msg::BoatState &msg) {
    std::unique_lock<std::mutex> l(boat_state_mutex_);
    *boat_state_ = msg;
  });
  RegisterHandler<msg::Vector3f>("wind", [this](const msg::Vector3f &msg) {
    wind_x_ = msg.x();
    wind_y_ = msg.y();
  });
  RegisterHandler<msg::HeadingCmd>("heading_cmd", [this](const msg::HeadingCmd &msg) {
    heading_ = msg.heading();
  });
}

void SimpleControl::Iterate() {
  std::unique_lock<std::mutex> l(boat_state_mutex_);

  // Set angle of attack to be ~.4
  float vx = boat_state_->vel().x();
  float vy = boat_state_->vel().y();
  float yaw = boat_state_->euler().yaw();
  float heel = boat_state_->euler().roll();
  VLOG(2) << "yaw: " << yaw << " heel: " << heel;

  float goal_heading = heading_;

  // For alphaw, if sailing straight into wind, =0, if wind is coming from port
  // (ie, on port tack), alphaw is positive, if coming from starboard, is
  // negative/between pi and 2pi.
  float alphaw = util::norm_angle(std::atan2(vy - wind_y_, vx - wind_x_) - yaw);
  float wind_source_dir = std::atan2(-wind_y_, -wind_x_);
  float alphasail = alphaw - boat_state_->internal().sail();
  // If we are on port tack, want alphasail > 0, if on starboard, alphasail < 0.
  static float goal = 0;
  goal = std::abs(alphaw) > 2.5 ? 1.5 : .4;
  if (std::abs(alphaw) < 2.8) {
    goal = alphaw > 0 ? goal : -goal;
  }
  VLOG(2) << "Alphaw: " << alphaw << " alphas: " << alphasail
          << " goals: " << goal;
  sail_msg_->set_vel(-util::norm_angle(goal - alphasail));

  ballast_msg_->set_vel(-0.5 * heel);

  float vel = std::sqrt(vx * vx + vy * vy);
  float max_rudder = vel < 0 ? 0.1 : 0.4;
  //float boat_heading = std::atan2(vy, vx);
  float cur_heading = yaw; // vel > 0.1 ? std::atan2(vy, vx) : yaw;
  float goal_rudder =
      std::min(std::max(-util::norm_angle(goal_heading - cur_heading /*yaw*/),
                        -max_rudder),
               max_rudder);
  VLOG(2) << "goalh: " << goal_heading << " goal_rudder: " << goal_rudder;
  rudder_msg_->set_vel(1. * (goal_rudder - boat_state_->internal().rudder()));
  sail_cmd_.send(sail_msg_);
  if (do_rudder_) {
    rudder_cmd_.send(rudder_msg_);
  }
  ballast_cmd_.send(ballast_msg_);
}

}  // control
}  // sailbot
