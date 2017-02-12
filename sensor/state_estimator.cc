#include "state_estimator.h"
#include "control/util.h"

namespace sailbot {
namespace control {

StateEstimator::StateEstimator()
    : Node(0.01), state_queue_("boat_state", true), wind_queue_("wind", true),
      euler_angles_(Vector3d::Zero()), omega_(Vector3d::Zero()),
      pos_(Vector3d::Zero()), vel_(Vector3d::Zero()),
      state_msg_(AllocateMessage<msg::BoatState>()),
      wind_msg_(AllocateMessage<msg::Vector3f>()) {
  // Vessel Heading
  RegisterHandler<msg::can::CANMaster>("can127250",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    if (msg.has_heading() && msg.heading().has_heading()) {
      euler_angles_[2] = msg.heading().heading();
    }
  });
  // Rate of Turn
  RegisterHandler<msg::can::CANMaster>("can127251",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    // TODO(james): Figure out what frame this is in.
    if (msg.has_rate_turn() && msg.rate_turn().has_rate()) {
      omega_[2] = msg.rate_turn().rate();
    }
  });
  // Attitude
  RegisterHandler<msg::can::CANMaster>("can127257",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    if (msg.has_attitude()) {
      if (msg.attitude().has_roll())
        euler_angles_[0] = msg.attitude().roll();
      if (msg.attitude().has_pitch())
        euler_angles_[1] = msg.attitude().pitch();
      if (msg.attitude().has_yaw())
        euler_angles_[2] = msg.attitude().yaw();
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
      // TODO(james): Confirm the frame of reference for COG.
      float heading = msg.cog_rapid_update().cog();
      vel_[0] = speed * std::cos(heading);
      vel_[1] = speed * std::sin(heading);
    }
  });
  // Wind Data
  RegisterHandler<msg::can::CANMaster>("can130306",
                                       [this](const msg::can::CANMaster &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    if (msg.has_wind_data() && msg.wind_data().has_wind_speed() &&
        msg.wind_data().has_wind_angle()) {
      const float speed = msg.wind_data().wind_speed();
      const float dir = msg.wind_data().wind_angle();
      wind_msg_->set_x(speed * std::cos(dir));
      wind_msg_->set_y(speed * std::sin(dir));
      wind_msg_->set_z(0);
      wind_queue_.send(wind_msg_);
    }
  });
  // Internal state data.
  RegisterHandler<msg::InternalBoatState>("internal_state",
                                       [this](const msg::InternalBoatState &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    *state_msg_->mutable_internal() = msg;
  });
}

void StateEstimator::Iterate() {
  std::unique_lock<std::mutex> lck(state_msg_mutex_);
  util::EigenToProto(omega_, state_msg_->mutable_omega());
  util::EigenToProto(pos_, state_msg_->mutable_pos());
  util::EigenToProto(vel_, state_msg_->mutable_vel());

  state_msg_->mutable_euler()->set_roll(euler_angles_(0));
  state_msg_->mutable_euler()->set_pitch(euler_angles_(1));
  state_msg_->mutable_euler()->set_yaw(euler_angles_(2));

  Eigen::Quaterniond q = util::RollPitchYawToQuat(euler_angles_);
  state_msg_->mutable_orientation()->set_w(q.w());
  state_msg_->mutable_orientation()->set_x(q.x());
  state_msg_->mutable_orientation()->set_y(q.y());
  state_msg_->mutable_orientation()->set_z(q.z());

  state_queue_.send(state_msg_);
}

}  // namespace sailbot
} // namespace control
