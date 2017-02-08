#include "sim_inter.h"
#include <Eigen/Geometry>

namespace sailbot {
namespace sim {

SimulatorNode::SimulatorNode()
    : Node(dt),
      // impl_(new SimulatorSaoud2013(dt)),
      impl_(new TrivialDynamics(dt)), sdot_(0), rdot_(0), bdot_(0),
      state_queue_("boat_state", true),
      state_msg_(AllocateMessage<msg::BoatState>()), wind_queue_("wind", true),
      wind_msg_(AllocateMessage<msg::Vector3f>()) {
    RegisterHandler<msg::SailCmd>(
        "sail_cmd",
        std::bind(&SimulatorNode::ProcessSail, this, std::placeholders::_1));
    RegisterHandler<msg::RudderCmd>(
        "rudder_cmd",
        std::bind(&SimulatorNode::ProcessRudder, this, std::placeholders::_1));
    RegisterHandler<msg::BallastCmd>(
        "ballast_cmd",
        std::bind(&SimulatorNode::ProcessBallast, this, std::placeholders::_1));
}

void SimulatorNode::ProcessSail(const msg::SailCmd& cmd) {
  if (cmd.has_vel() || std::isnan(cmd.vel()))
    sdot_ = cmd.vel();
}

void SimulatorNode::ProcessRudder(const msg::RudderCmd& cmd) {
  if (cmd.has_vel() || std::isnan(cmd.vel()))
    rdot_ = cmd.vel();
}

void SimulatorNode::ProcessBallast(const msg::BallastCmd& cmd) {
  if (cmd.has_vel() || std::isnan(cmd.vel()))
    bdot_ = cmd.vel();
}

void SimulatorNode::Iterate() {
  static float time = 0;
  VLOG(2) << "Time: " << (time += dt);
  VLOG(1) << "s, r: " << sdot_ << ", " << rdot_;
  //if (time > .05) std::exit(0);
  impl_->Update(sdot_, rdot_, bdot_);
  Eigen::Vector3d omega = impl_->get_omega();
  Eigen::Vector3d x = impl_->get_x();
  Eigen::Vector3d v = impl_->get_v();
  Eigen::Quaternion<double> rot(impl_->get_RBI());

  Eigen::Vector3d rollpitchyaw = GetRollPitchYaw(impl_->get_RBI());

  msg::Vector3f *pos = state_msg_->mutable_pos();
  pos->set_x(x(0));
  pos->set_y(x(1));
  pos->set_z(x(2));

  msg::Vector3f *vel = state_msg_->mutable_vel();
  vel->set_x(v(0));
  vel->set_y(v(1));
  vel->set_z(v(2));

  msg::Vector3f *wmsg = state_msg_->mutable_omega();
  wmsg->set_x(omega(0));
  wmsg->set_y(omega(1));
  wmsg->set_z(omega(2));

  msg::Quaternion *qmsg = state_msg_->mutable_orientation();
  qmsg->set_w(rot.w());
  qmsg->set_x(rot.x());
  qmsg->set_y(rot.y());
  qmsg->set_z(rot.z());

  msg::EulerAngles *euler = state_msg_->mutable_euler();
  euler->set_roll(rollpitchyaw(0, 0));
  euler->set_pitch(rollpitchyaw(1, 0));
  euler->set_yaw(rollpitchyaw(2, 0));

  state_msg_->mutable_internal()->set_sail(impl_->get_deltas());
  state_msg_->mutable_internal()->set_rudder(impl_->get_deltar());
  state_msg_->mutable_internal()->set_ballast(impl_->get_deltab());

  state_queue_.send(state_msg_);
}

}  // sim
}  // sailbot
