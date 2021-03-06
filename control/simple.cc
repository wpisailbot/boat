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
      heel_msg_(AllocateMessage<msg::HeelCmd>()),
      rigid_msg_(AllocateMessage<msg::RigidWingCmd>()),
      boat_state_(AllocateMessage<msg::BoatState>()),
      consts_msg_(AllocateMessage<msg::ControllerConstants>()),
      heading_(0),
      sail_cmd_("sail_cmd", true),
      rudder_cmd_("rudder_cmd", true),
      heel_cmd_("heel_cmd", true),
      rigid_cmd_("rigid_wing_cmd", true),
      consts_queue_("control_consts", true) {

  consts_msg_->set_max_rudder(0.5);
  consts_msg_->set_rudder_kp(0.5);
  consts_msg_->set_rudder_ki(0.02);
  consts_msg_->set_winch_kp(13);
  consts_msg_->set_sail_heel_k(0);

  consts_msg_->set_nominal_heel(0.0);

  consts_msg_->set_rigid_port_servo_pos(15);
  consts_msg_->set_rigid_starboard_servo_pos(85);
  {
    std::unique_lock<std::mutex> l(consts_mutex_);
    consts_queue_.send(consts_msg_);
  }

  rigid_msg_->set_state(msg::RigidWingCmd_WingState_MANUAL);
  rigid_msg_->set_heel(0);
  rigid_msg_->set_max_heel(1);
  RegisterHandler<msg::BoatState>("boat_state", [this](const msg::BoatState &msg) {
    std::unique_lock<std::mutex> l(boat_state_mutex_);
    *boat_state_ = msg;
  });
  RegisterHandler<msg::ControllerConstants>(
      "control_consts", [this](const msg::ControllerConstants &msg) {
        if (msg.has_max_rudder() && msg.has_rudder_kp() && msg.has_winch_kp() &&
            msg.has_sail_heel_k() && msg.has_rigid_port_servo_pos() &&
            msg.has_rigid_starboard_servo_pos() && msg.has_nominal_heel()) {
          std::unique_lock<std::mutex> l(consts_mutex_);
          *consts_msg_ = msg;
        }
      });
  RegisterHandler<msg::Vector3f>("wind", [this](const msg::Vector3f &msg) {
    wind_x_ = msg.x();
    wind_y_ = msg.y();
  });
  RegisterHandler<msg::HeadingCmd>("heading_cmd", [this](const msg::HeadingCmd &msg) {
    if (msg.has_heading()) {
      heading_ = msg.heading();
    }
    if (msg.has_extra_sail()) {
      extra_sail_ = msg.extra_sail();
    }
    rudder_integrator_ = 0.0;
    VLOG(2) << "Got heading cmd: " << heading_.load();
  });
  RegisterHandler<msg::ControlMode>("control_mode",
                                    [this](const msg::ControlMode &mode) {
    if (mode.has_rigid_mode()) {
      switch (mode.rigid_mode()) {
      case msg::ControlMode::AUTO:
        auto_rigid_wing_ = true;
        break;
      default:
        auto_rigid_wing_ = false;
        break;
      }
    }
  });
  RegisterHandler<msg::SBUS>("sbus_value", [this](const msg::SBUS &sbus) {
    // TODO(james): Have separate mutex or atomic variable just for rigid wing.
    std::unique_lock<std::mutex> l(boat_state_mutex_);
    rigid_msg_->set_servo_pos(sbus.channel(1) * 100. / 2050.);
  });
}

void SimpleControl::Iterate() {
  ++counter_;
  std::unique_lock<std::mutex> l(boat_state_mutex_);
  std::unique_lock<std::mutex> lc(consts_mutex_);

  // Set angle of attack to be ~.4
  float vx = boat_state_->vel().x();
  float vy = boat_state_->vel().y();
  float yaw = boat_state_->euler().yaw();
  float heel = boat_state_->euler().roll();
  float heel_rate = boat_state_->omega().x();
  VLOG(2) << "yaw: " << yaw << " heel: " << heel;

  float goal_heading = heading_;
  float cur_heading = yaw; // vel > 0.1 ? std::atan2(vy, vx) : yaw;
  double heading_err = util::norm_angle(goal_heading - cur_heading);

  // For alphaw, if sailing straight into wind, =0, if wind is coming from port
  // (ie, on port tack), alphaw is positive, if coming from starboard, is
  // negative/between pi and 2pi.
//  float alphaw = util::norm_angle(std::atan2(vy - wind_y_, vx - wind_x_) - yaw);
  float alphaw = util::norm_angle(std::atan2(wind_y_, -wind_x_));
  float wind_source_dir = std::atan2(-wind_y_, -wind_x_);
  //float alphasail = util::norm_angle(alphaw - boat_state_->internal().sail());
  float cursail = boat_state_->internal().sail();
  double turning_sail =
      (alphaw > 0.0 ? 1.0 : -1.0) * 0.9 * util::Clip(heading_err, -0.5, 0.5);
  // If we are on port tack, want alphasail > 0, if on starboard, alphasail < 0.
  float goal = std::min(std::max(std::abs(alphaw)- 0.8, 0.), 1.5) +
               consts_msg_->sail_heel_k() * std::abs(heel)
               + extra_sail_
               + turning_sail;
  VLOG(2) << "Alphaw: " << alphaw << " alphas: " << cursail
          << " goals: " << goal;
  // TODO(james): Temporary for testing:
  //goal = std::abs(goal_heading);
  //alphasail = boat_state_->internal().sail();
  float sail_err = util::norm_angle(goal - cursail);
  sail_msg_->set_voltage(consts_msg_->winch_kp() * sail_err /
                         std::sqrt(std::abs(sail_err)));
  sail_msg_->set_pos(goal);
  if (auto_rigid_wing_) {
    rigid_msg_->set_servo_pos(wind_source_dir > 0
                                  ? consts_msg_->rigid_port_servo_pos()
                                  : consts_msg_->rigid_starboard_servo_pos());
  }

  float vel = std::sqrt(vx * vx + vy * vy);
  double max_rudder =
      vel < 0 ? 0.3 : (vel < 1.0 ? consts_msg_->max_rudder()
                                 : 0.8 * consts_msg_->max_rudder());
  //float boat_heading = std::atan2(vy, vx);
  rudder_integrator_ =
      util::Clip(rudder_integrator_ + heading_err * dt, -1.5, 1.5);
  double goal_rudder =
      -util::Clip(consts_msg_->rudder_kp() * heading_err +
                      consts_msg_->rudder_ki() * rudder_integrator_,
                  -max_rudder, max_rudder);
  VLOG(2) << "goalh: " << goal_heading << " goal_rudder: " << goal_rudder;
  rudder_msg_->set_pos(goal_rudder);
  rudder_msg_->set_vel(1. * (goal_rudder - boat_state_->internal().rudder()));
  sail_cmd_.send(sail_msg_);
  if (auto_rigid_wing_ && (counter_ % int(.1 / dt)) == 0)
    rigid_cmd_.send(rigid_msg_);
  if (do_rudder_) {
    rudder_cmd_.send(rudder_msg_);
  }
  if (std::abs(alphaw) > 1.4) {
    // Don't send new heel command when we are sailing pretty much downwind;
    // prevents oscillations which some people don't seem to like.
    double heel_goal = consts_msg_->nominal_heel() * util::Sign(wind_source_dir) +
                       0 * util::Clip(1.0 * heading_err, -0.25, 0.25);
    heel_msg_->set_heel(heel_goal);
    heel_cmd_.send(heel_msg_);
  }
  if (counter_ % 100 == 0) {
    consts_queue_.send(consts_msg_);
  }
}

}  // control
}  // sailbot
