#include "simple.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace sailbot {
namespace control {

namespace {
  float norm_angle(float a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
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
  float alphasail = alphaw - boat_state_->internal().sail();
  // If we are on port tack, want alphasail > 0, if on starboard, alphasail < 0.
  float goal = alphaw > 0 ? .4 : -.4;
  VLOG(2) << "Alphaw: " << alphaw << " alphas: " << alphasail
          << " goals: " << goal;
  sail_msg_->set_vel(-norm_angle(goal - alphasail));

  float goal_heading = .1;
  float max_rudder = .4;
  float boat_heading = std::atan2(vy, vx);
  float goal_rudder = std::min(
      std::max(-(goal_heading - boat_heading), -max_rudder), max_rudder);
  VLOG(2) << "goalh: " << goal_heading << " goal_rudder: " << goal_rudder;
  rudder_msg_->set_vel(goal_rudder - boat_state_->internal().rudder());
  VLOG(1) << "sdot " << sail_msg_->vel() << " rdot " << rudder_msg_->vel();
  sail_cmd_.send(sail_msg_);
  rudder_cmd_.send(rudder_msg_);
}

}  // control
}  // sailbot
