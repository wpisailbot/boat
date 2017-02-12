#include "fake_airmar.h"

namespace sailbot {
namespace sim {

FakeAirmar::FakeAirmar()
    : Node(dt), heading_("can127250", true), rate_turn_("can127251", true),
      attitude_("can127257", true), pos_rapid_update_("can129025", true),
      cog_rapid_update_("can129026", true), wind_data_("can130306", true),
      counter_(0) {
  RegisterHandler<msg::BoatState>("sim_true_boat_state",
                                  [this](const msg::BoatState &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    state_ = msg;
  });
  RegisterHandler<msg::Vector3f>("sim_true_wind",
                                 [this](const msg::Vector3f &msg) {
    std::unique_lock<std::mutex> lck(state_msg_mutex_);
    wind_ = msg;
  });
}

void FakeAirmar::Iterate() {
  msg::can::CANMaster out;

  out.mutable_heading()->set_heading(state_.euler().yaw());
  heading_.send(&out);
  out.clear_heading();

  // TODO(james): Determine proper frame for rate of turn.
  out.mutable_rate_turn()->set_rate(state_.omega().z());
  rate_turn_.send(&out);
  out.clear_rate_turn();

  // Attitude only gets sent out at 1 Hz...
//  if ((counter_ % int(1 / dt)) == 0) {
    out.mutable_attitude()->set_roll(state_.euler().roll());
    out.mutable_attitude()->set_pitch(state_.euler().pitch());
    out.mutable_attitude()->set_yaw(state_.euler().yaw());
    attitude_.send(&out);
    out.clear_attitude();
//  }

  out.mutable_pos_rapid_update()->set_lon(state_.pos().x());
  out.mutable_pos_rapid_update()->set_lat(state_.pos().y());
  pos_rapid_update_.send(&out);
  out.clear_pos_rapid_update();

  float vx = state_.vel().x();
  float vy = state_.vel().y();
  out.mutable_cog_rapid_update()->set_sog(std::sqrt(vx * vx + vy * vy));
  out.mutable_cog_rapid_update()->set_cog(std::atan2(vy, vx));
  cog_rapid_update_.send(&out);
  out.clear_cog_rapid_update();

  out.mutable_wind_data()->set_wind_speed(
      std::sqrt(wind_.x() * wind_.x() + wind_.y() * wind_.y()));
  out.mutable_wind_data()->set_wind_angle(std::atan2(wind_.y(), wind_.x()));
  wind_data_.send(&out);
  out.clear_wind_data();

  ++counter_;
}

}  // namespace sim
}  // namespace sailbot
