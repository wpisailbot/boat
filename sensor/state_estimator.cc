#include "state_estimator.h"
#include "control/util.h"

namespace sailbot {
namespace control {

StateEstimator::StateEstimator()
    : Node(0.01), state_queue_("boat_state", true), wind_queue_("wind", true),
      true_wind_queue_("true_wind", true), euler_angles_(Vector3d::Zero()),
      omega_(Vector3d::Zero()), pos_(Vector3d::Zero()), vel_(Vector3d::Zero()),
      state_msg_(AllocateMessage<msg::BoatState>()),
      wind_msg_(AllocateMessage<msg::Vector3f>()) {
  // Vessel Heading
  RegisterHandler<msg::can::CANMaster>("can127250",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    if (msg.has_heading() && msg.heading().has_heading()) {
      euler_angles_[2] = -msg.heading().heading() + M_PI / 2.;
    }
  });
  // Rate of Turn
  RegisterHandler<msg::can::CANMaster>("can127251",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    // TODO(james): Figure out what frame this is in.
    if (msg.has_rate_turn() && msg.rate_turn().has_rate()) {
      omega_[2] = -msg.rate_turn().rate();
    }
  });
  // Attitude
  RegisterHandler<msg::can::CANMaster>("can127257",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    if (msg.has_attitude()) {
      //if (msg.attitude().has_roll())
      //  euler_angles_[0] = msg.attitude().roll();
      if (msg.attitude().has_pitch())
        euler_angles_[1] = msg.attitude().pitch();
      //if (msg.attitude().has_yaw())
      //  euler_angles_[2] = msg.attitude().yaw();
    }
  });
  // Position Update
  RegisterHandler<msg::can::CANMaster>("can129025",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    if (msg.has_pos_rapid_update() && msg.pos_rapid_update().has_lat() &&
        msg.pos_rapid_update().has_lon()) {
      pos_[0] = msg.pos_rapid_update().lon();
      pos_[1] = msg.pos_rapid_update().lat();
    }
  });
  // Velocity Update
  RegisterHandler<msg::can::CANMaster>("can129026",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    if (msg.has_cog_rapid_update() && msg.cog_rapid_update().has_sog() &&
        msg.cog_rapid_update().has_cog()) {
      float speed = msg.cog_rapid_update().sog();
      float heading = M_PI / 2. - msg.cog_rapid_update().cog();
      vel_[0] = speed * std::cos(heading);
      vel_[1] = speed * std::sin(heading);
    }
  });
  // Wind Data
  RegisterHandler<msg::can::CANMaster>("can130306",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    if (msg.has_wind_data() && msg.wind_data().has_wind_speed() &&
        msg.wind_data().has_wind_angle() && msg.wind_data().has_reference()) {
      bool true_wind = msg.wind_data().reference() !=
                       msg::can::WindData_WIND_REFERENCE_APPARENT;
      // Airmar provides direction wind is coming from, not where it is
      // and uses compass headings, not real headings.
      // compass headings are (a) left-handed and (b) North-zeroed
      float speed = -msg.wind_data().wind_speed();
      float dir = -msg.wind_data().wind_angle();
      if (true_wind) {
        dir += M_PI / 2.; // Compensate for North = 0 vs. East = 0.
      }
      wind_msg_->set_x(speed * std::cos(dir));
      wind_msg_->set_y(speed * std::sin(dir));
      wind_msg_->set_z(0);
      if (!true_wind) {
        wind_queue_.send(wind_msg_);
      } else {
        true_wind_queue_.send(wind_msg_);
      }
    }
  });
  // Internal state data.
  RegisterHandler<msg::InternalBoatState>("internal_state",
                                       [this](const msg::InternalBoatState &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    if (msg.has_sail()) {
      state_msg_->mutable_internal()->set_sail(msg.sail());
    }
    if (msg.has_rudder()) {
      state_msg_->mutable_internal()->set_rudder(msg.rudder());
    }
    if (msg.has_ballast()) {
      state_msg_->mutable_internal()->set_ballast(msg.ballast());
      double dt =
          std::chrono::nanoseconds(Time() - last_ballast_time_).count() / 1e9;
      dt = std::max(dt, 0.001); // Prevent divide-by-zero

      double ballast_angle = msg.ballast();
      state_msg_->mutable_internal()->set_ballast(ballast_angle);
      double ballastdot = (ballast_angle - last_ballast_) / dt;
      ballastdot =
          state_msg_->internal().ballastdot() * 0.5 + 0.5 * ballastdot;
      state_msg_->mutable_internal()->set_ballastdot(ballastdot);

      last_ballast_ = ballast_angle;
      last_ballast_time_ = Time();
    }
  });
  // State from inclinometer + ballast encoder:
  RegisterHandler<msg::can::CANMaster>("can65285",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    if (msg.has_ballast_state() && msg.ballast_state().has_heel()) {
      double dt =
          std::chrono::nanoseconds(Time() - last_inclinometer_time_).count() / 1e9;
      dt = std::max(dt, 0.001); // Prevent divide-by-zero
      // TODO(james): Filter
      double heel = msg.ballast_state().heel();
      euler_angles_[0] = heel;
      // Add in filter to prevent silly velocity measurements
      omega_[0] += 0.5 * ((heel - last_inclinometer_) / dt - omega_[0]);

      last_inclinometer_ = heel;
      last_inclinometer_time_ = Time();
    }
  });
}

void StateEstimator::Iterate() {
  std::unique_lock<std::mutex> lck(state_msg_mutex_);
  util::EigenToProto(omega_, state_msg_->mutable_omega());
  util::EigenToProtod(pos_, state_msg_->mutable_pos());
  util::EigenToProto(vel_, state_msg_->mutable_vel());

  state_msg_->mutable_euler()->set_roll(euler_angles_[0]);
  state_msg_->mutable_euler()->set_pitch(euler_angles_[1]);
  state_msg_->mutable_euler()->set_yaw(euler_angles_[2]);

  Eigen::Quaterniond q = util::RollPitchYawToQuat(euler_angles_);
  state_msg_->mutable_orientation()->set_w(q.w());
  state_msg_->mutable_orientation()->set_x(q.x());
  state_msg_->mutable_orientation()->set_y(q.y());
  state_msg_->mutable_orientation()->set_z(q.z());

  state_queue_.send(state_msg_);
}

}  // namespace sailbot
} // namespace control
