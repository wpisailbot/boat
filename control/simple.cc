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
      rigid_msg_(AllocateMessage<msg::RigidWingCmd>()),
      boat_state_(AllocateMessage<msg::BoatState>()),
      heading_(0),
      sail_cmd_("sail_cmd", true),
      rudder_cmd_("rudder_cmd", true),
      ballast_cmd_("ballast_cmd", true),
      rigid_cmd_("rigid_wing_cmd", true) {
  rigid_msg_->set_state(msg::RigidWingCmd_WingState_MANUAL);
  rigid_msg_->set_heel(0);
  rigid_msg_->set_max_heel(1);
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
    VLOG(2) << "Got heading cmd: " << heading_.load();
  });
}

void SimpleControl::Iterate() {
  ++counter_;
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
//  float alphaw = util::norm_angle(std::atan2(vy - wind_y_, vx - wind_x_) - yaw);
  float alphaw = std::abs(util::norm_angle(std::atan2(wind_y_, -wind_x_)));
  float wind_source_dir = std::atan2(-wind_y_, -wind_x_);
  //float alphasail = util::norm_angle(alphaw - boat_state_->internal().sail());
  float cursail = boat_state_->internal().sail();
  // If we are on port tack, want alphasail > 0, if on starboard, alphasail < 0.
  float goal = std::min(std::max(alphaw - 0.4, 0.), 1.5);
  VLOG(2) << "Alphaw: " << alphaw << " alphas: " << cursail
          << " goals: " << goal;
  // TODO(james): Temporary for testing:
  //goal = std::abs(goal_heading);
  //alphasail = boat_state_->internal().sail();
  float sail_err = util::norm_angle(goal - cursail);
  sail_msg_->set_voltage(3 * sail_err / std::sqrt(std::abs(sail_err)));
  sail_msg_->set_pos(goal);
  rigid_msg_->set_servo_pos(wind_source_dir > 0 ? 30 : 70);

  ballast_msg_->set_vel(-0.5 * heel);

  float vel = std::sqrt(vx * vx + vy * vy);
  float max_rudder = vel < 0 ? 0.1 : (vel < 0.5 ? 0.25 : 0.7);
  //float boat_heading = std::atan2(vy, vx);
  float cur_heading = yaw; // vel > 0.1 ? std::atan2(vy, vx) : yaw;
  float goal_rudder =
      std::min(std::max(-util::norm_angle(goal_heading - cur_heading /*yaw*/),
                        -max_rudder),
               max_rudder);
  VLOG(2) << "goalh: " << goal_heading << " goal_rudder: " << goal_rudder;
  rudder_msg_->set_pos(goal_rudder);
  rudder_msg_->set_vel(1. * (goal_rudder - boat_state_->internal().rudder()));
  sail_cmd_.send(sail_msg_);
  if ((counter_ % int(.1 / dt)) == 0)
    rigid_cmd_.send(rigid_msg_);
  if (do_rudder_) {
    rudder_cmd_.send(rudder_msg_);
  }
  ballast_cmd_.send(ballast_msg_);
}

}  // control
}  // sailbot
