#include "ballast.h"
#include "control/actuator_cmd.pb.h"
#include "util/node.h"

namespace sailbot {
namespace control {

BallastControl::BallastControl()
    : Node(dt), ballast_msg_(AllocateMessage<msg::BallastCmd>()),
      consts_msg_(AllocateMessage<msg::ControllerConstants>()),
      ballast_cmd_("ballast_cmd", true), consts_queue_("control_consts", true) {
  consts_msg_->set_ballast_heel_kp(1.0);
  consts_msg_->set_ballast_heel_ki(2.0);
  consts_msg_->set_ballast_heel_kd(0.0);
  consts_msg_->set_ballast_heel_kff_goal(4.0);

  consts_msg_->set_ballast_arm_kp(40.0);
  consts_msg_->set_ballast_arm_kd(0.0);
  consts_msg_->set_ballast_arm_kff_arm(0.0);
  consts_msg_->set_ballast_arm_kff_heel(0.0);

  {
    std::unique_lock<std::mutex> l(consts_mutex_);
    consts_queue_.send(consts_msg_);
  }

  RegisterHandler<msg::ControllerConstants>(
      "control_consts", [this](const msg::ControllerConstants &msg) {
        if (msg.has_ballast_heel_kp() && msg.has_ballast_heel_ki() &&
            msg.has_ballast_heel_kd() && msg.has_ballast_arm_kp() &&
            msg.has_ballast_arm_kd() && msg.has_ballast_arm_kff_arm() &&
            msg.has_ballast_arm_kff_heel() && msg.has_ballast_heel_kff_goal()) {
          std::unique_lock<std::mutex> l(consts_mutex_);
          *consts_msg_ = msg;
          heel_error_integrator_ = 0.0;
        }
      });

  RegisterHandler<msg::ControlMode>("control_mode",
                                    [this](const msg::ControlMode &msg) {
    if (msg.has_ballast_mode()) {
      heel_error_integrator_ = 0.0;
    }
  });

  RegisterHandler<msg::HeelCmd>("heel_cmd", [this](const msg::HeelCmd &msg) {
    if (msg.has_heel()) {
      if (msg.heel() == 0) {
        heel_error_integrator_ = 0.0;
      } else if (util::Sign(msg.heel()) != util::Sign(heel_goal_.load())) {
        heel_error_integrator_ = -heel_error_integrator_;
      }
      heel_goal_ = msg.heel();
    }
  });

  RegisterHandler<msg::BoatState>("boat_state",
                                  [this](const msg::BoatState &msg) {
    heel_ = msg.euler().roll();
    heel_dot_ = msg.omega().x();
    ballast_ = msg.internal().ballast();
    ballast_dot_ = msg.internal().ballastdot();
  });
}

void BallastControl::Iterate() {
  double heel_error = heel_goal_ - heel_;
  double dheel_error = -heel_dot_;
  heel_error_integrator_ = heel_error_integrator_ + heel_error * dt;
  heel_error_integrator_ =
      util::Clip((double)heel_error_integrator_, -1.0, 1.0);

  double ballast_goal =
      consts_msg_->ballast_heel_kp() * heel_error +
      consts_msg_->ballast_heel_ki() * heel_error_integrator_ +
      consts_msg_->ballast_heel_kd() * dheel_error +
      consts_msg_->ballast_heel_kff_goal() * heel_goal_;
  ballast_goal = util::Clip(ballast_goal, -1.2, 1.2);

  double ballast_error = ballast_goal - ballast_;
  double dballast_error = -ballast_dot_;
  double ballast_error_deadband = 0.00;
  // Create deadband around ballast goal, and ensure
  // that error is continuous
  if (ballast_error > 0.0) {
    ballast_error = std::max(0.0, ballast_error - ballast_error_deadband);
  } else {
    ballast_error = std::min(0.0, ballast_error + ballast_error_deadband);
  }
  double ballast_voltage = consts_msg_->ballast_arm_kp() * ballast_error +
                           consts_msg_->ballast_arm_kd() * dballast_error -
                           consts_msg_->ballast_arm_kff_arm() * ballast_ -
                           consts_msg_->ballast_arm_kff_heel() * heel_;
  ballast_voltage = util::Clip(ballast_voltage, -12.0, 12.0);
  double voltage_deadband = last_voltage_ == 0 ? 2.5 : 1.0;
  if (std::abs(ballast_error) < 0.001 || std::abs(ballast_) > 1.5 ||
      std::abs(ballast_voltage) < voltage_deadband) {
    // If sufficiently close, then tack advantage of non-backdrivability:
    // Also, if ballast position is clearly absurd, don't apply voltage
    ballast_voltage = 0.0;
  }

  last_voltage_ = ballast_voltage;
  ballast_msg_->set_voltage(ballast_voltage);
  ballast_cmd_.send(ballast_msg_);
  if (++counter_ % 110 == 0) {
    consts_queue_.send(consts_msg_);
  }
}

}  // control
}  // sailbot
