#include "sim_inter.h"
#include <Eigen/Geometry>
#include <gflags/gflags.h>

DEFINE_double(starttime, 0, "Process time at which to start the simulation");
DEFINE_double(endtime, 9999, "Process time at which to end the simulation");

namespace {
constexpr double kAirmarOffset = -.0;

};

namespace sailbot {
namespace sim {

SimulatorNode::SimulatorNode(double reset_period, bool set_pos)
    : Node(dt),
      reset_period_(reset_period),
      set_pos_(set_pos),
      // impl_(new SimulatorSaoud2013(dt)),
      impl_(new TrivialDynamics(dt)), sdot_(0), rdot_(0), bdot_(0),
      state_queue_("sim_true_boat_state", true),
      state_msg_(AllocateMessage<msg::BoatState>()),
      wind_queue_("sim_true_wind", true),
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
    RegisterHandler<msg::Vector3f>("sim_set_wind",
                                   [this](const msg::Vector3f &msg) {
      set_wind(msg.x(), msg.y(), msg.z());
      // Don't actually need to correct for airmar, because this is the true,
      // not apparent, wind.
      // float ang = std::atan2(msg.y(), msg.x());
      // float speed = std::sqrt(msg.x() * msg.x() + msg.y() * msg.y());
      // ang -= kAirmarOffset; // not sure of sign
      // set_wind(ang, speed);
    });
    // For when doing log replay, retrieve logged state.
    // And set sail appropriately.
    RegisterHandler<msg::BoatState>("orig_boat_state",
                                    [this](const msg::BoatState &msg) {
      std::unique_lock<std::mutex> lck(last_state_mutex_);
      last_state_ = msg;
      last_state_.mutable_euler()->set_yaw(msg.euler().yaw() - kAirmarOffset);

      deltas_ = msg.internal().sail();
    });
}

void SimulatorNode::ProcessSail(const msg::SailCmd& cmd) {
  if (cmd.has_vel() && !std::isnan(cmd.vel())) {
    sdot_ = cmd.vel();
  } else if (cmd.has_pos() && !std::isnan(cmd.pos())) {
    double pos = cmd.pos();
    sdot_ = impl_->get_sdot_for_goal(pos);
  }
}

void SimulatorNode::ProcessRudder(const msg::RudderCmd& cmd) {
  deltar_ = cmd.pos();
  if (cmd.has_vel() && !std::isnan(cmd.vel())) {
    rdot_ = cmd.vel();
  } else if (cmd.has_pos() && !std::isnan(cmd.pos())) {
    rdot_ = impl_->get_rdot_for_goal(cmd.pos());
  }
}

void SimulatorNode::ProcessBallast(const msg::BallastCmd& cmd) {
  if (cmd.has_vel() && !std::isnan(cmd.vel()))
    bdot_ = cmd.vel();
}

void SimulatorNode::Run() {
  std::thread t_internal(&FakeInternalSensors::Run, &internal_sensors_);
  std::thread t_airmar(&FakeAirmar::Run, &fake_airmar_);
  Node::Run();
  t_airmar.join();
  t_internal.join();
}

void SimulatorNode::Iterate() {
  const double systime =
      std::chrono::nanoseconds(util::monotonic_clock::now().time_since_epoch())
          .count() /
      1e9;
  if (systime < FLAGS_starttime) {
    return;
  }
  if (systime > FLAGS_endtime) {
    util::RaiseShutdown();
    return;
  }
  if (reset_period_ > 0 && systime > next_reset_) {
    started_ = false;
    next_reset_ = systime + reset_period_;
  }
  if (!started_) {
    std::unique_lock<std::mutex> lck(last_state_mutex_);
    impl_->set_from_state(last_state_);
    if (last_state_.has_pos()) {
      fake_airmar_.set_gps_zero(last_state_.pos().y(), last_state_.pos().x());
    } else {
      fake_airmar_.set_gps_zero(38.9816688, -76.47591338);
    }
    started_ = true;
  }
  usleep(dt * 1e6 / 2.);
  static float time = 0;
  VLOG(2) << "Time: " << (time += dt);
  VLOG(1) << "s, r: " << sdot_ << ", " << rdot_;
  //if (time > .05) std::exit(0);
  if (set_pos_) {
    impl_->Update(deltas_, deltar_);
  } else {
    impl_->Update(sdot_, rdot_, bdot_);
  }
  Eigen::Vector3d omega = impl_->get_omega();
  Eigen::Vector3d x = impl_->get_x();
  Eigen::Vector3d v = impl_->get_v();
  Eigen::Quaternion<double> rot(impl_->get_RBI());

  Eigen::Vector3d rollpitchyaw = util::GetRollPitchYaw(impl_->get_RBI());

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

  state_msg_->mutable_internal()->set_sail(std::abs(impl_->get_deltas()));
  // XXX(james): Resolve abs vs. not for deltas
  state_msg_->mutable_internal()->set_sail(impl_->get_deltas());
  state_msg_->mutable_internal()->set_rudder(impl_->get_deltar());
  state_msg_->mutable_internal()->set_ballast(impl_->get_deltab());
  state_msg_->mutable_internal()->set_saildot(sdot_);
  state_msg_->mutable_internal()->set_rudderdot(rdot_);
  state_msg_->mutable_internal()->set_ballastdot(bdot_);

  state_queue_.send(state_msg_);
}

}  // sim
}  // sailbot
