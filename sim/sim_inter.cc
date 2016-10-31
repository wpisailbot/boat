#include "sim_inter.h"
#include <eigen3/Eigen/Geometry>

namespace sailbot {
namespace sim {

void SimulatorNode::ProcessSail(const msg::SailCmd& cmd) {
  sdot_ = cmd.vel();
}

void SimulatorNode::ProcessRudder(const msg::RudderCmd& cmd) {
  rdot_ = cmd.vel();
}

void SimulatorNode::Iterate() {
    impl_.Update(sdot_, rdot_);
    Eigen::Vector3d omega = impl_.get_omega();
    Eigen::Vector3d x = impl_.get_x();
    Eigen::Vector3d v = impl_.get_v();
    Eigen::Quaternion<double> rot(impl_.get_RBI());

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

    state_msg_->mutable_internal()->set_sail(impl_.get_deltas());
    state_msg_->mutable_internal()->set_rudder(impl_.get_deltar());

    state_queue_.send(state_msg_);
    // TODO: Display/show results of simulation.
}

}  // sim
}  // sailbot
