#include "fake_airmar.h"
#include "control/util.h"

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

namespace {
  using util::Normal;
}

void FakeAirmar::Iterate() {
  msg::can::CANMaster out;

  // Reverse sign of yaw because headings are compass headings
  out.mutable_heading()->set_heading(-state_.euler().yaw() + Normal(0, 0.05));
  heading_.send(&out);
  out.clear_heading();

  // TODO(james): Determine proper frame for rate of turn.
  out.mutable_rate_turn()->set_rate(state_.omega().z() + Normal(0, 0.05));
  rate_turn_.send(&out);
  out.clear_rate_turn();

  // Attitude only gets sent out at 1 Hz...
  if ((counter_ % int(1 / dt)) == 0) {
    out.mutable_attitude()->set_roll(state_.euler().roll() + Normal(0, 0.05));
    out.mutable_attitude()->set_pitch(state_.euler().pitch() + Normal(0, 0.05));
    // TODO(james): Check whether this yaw needs to be reversed.
    out.mutable_attitude()->set_yaw(state_.euler().yaw() + Normal(0, 0.05));
    attitude_.send(&out);
    out.clear_attitude();
  }

  double x = state_.pos().x() + Normal(0, .5);
  double y = state_.pos().y() + Normal(0, .5);
  double lat = y / 111054. + 41;
  double lon = x / 84135. - 71;
  out.mutable_pos_rapid_update()->set_lon(lon);
  out.mutable_pos_rapid_update()->set_lat(lat);
  pos_rapid_update_.send(&out);
  out.clear_pos_rapid_update();

  float vx = state_.vel().x();
  float vy = state_.vel().y();
  out.mutable_cog_rapid_update()->set_sog(std::sqrt(vx * vx + vy * vy) +
                                          Normal(0, 0.1));
  out.mutable_cog_rapid_update()->set_cog(std::atan2(vy, vx) + Normal(0, 0.05));
  cog_rapid_update_.send(&out);
  out.clear_cog_rapid_update();

  out.mutable_wind_data()->set_wind_speed(
      std::sqrt(wind_.x() * wind_.x() + wind_.y() * wind_.y()) +
      Normal(0, 0.1));
  // Invert x/y to properly spoof airmar wind from, rather than wind dir.
  out.mutable_wind_data()->set_wind_angle(std::atan2(-wind_.y(), -wind_.x()) +
                                          Normal(0, 0.05));
  out.mutable_wind_data()->set_reference(msg::can::WindData_WIND_REFERENCE_TRUE_NORTH_REF);
  wind_data_.send(&out);
  // And now do apparent wind...
  double wx = wind_.x() - vx;
  double wy = wind_.y() - vy;
  // Add M_PI to handle to vs. from wind dir.
  double alphaw =
      util::norm_angle(M_PI + std::atan2(wy, wx) - state_.euler().yaw());
  out.mutable_wind_data()->set_wind_speed(std::sqrt(wx * wx + wy * wy));
  out.mutable_wind_data()->set_wind_angle(alphaw);
  out.mutable_wind_data()->set_reference(msg::can::WindData_WIND_REFERENCE_APPARENT);
  wind_data_.send(&out);
  out.clear_wind_data();

  ++counter_;
}

} // namespace sim
} // namespace sailbot
