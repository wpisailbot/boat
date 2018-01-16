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
  float Normal(float std) {
    float r = util::Normal(0, std);
    const int Nstd = 3;
//    return std::max(std::min(r, r + Nstd * std), r - Nstd * std);
    return 0.0;
  }
  float CalcNew(const float last, const float truth, const float kT) {
    const float diff = util::norm_angle(truth - last);
    return util::norm_angle(last + (1 - kT) * diff);
  }
}

void FakeAirmar::Iterate() {
  std::unique_lock<std::mutex> lck(state_msg_mutex_);
  if (!state_.has_euler() || !wind_.has_x()) {
    return;
  }
  constexpr float kTdt = 0 * std::exp(dt * kTContinuous);
  constexpr float kT1Hz = 0 * std::exp(kTContinuous);
  msg::can::CANMaster out;

  // Reverse sign of yaw because headings are compass headings
  const float yaw = state_.euler().yaw();
  const float heading = M_PI / 2. - yaw;
  out.mutable_heading()->set_heading(
      CalcNew(last_.heading().heading(), heading, kTdt));
  heading_.send(&out);
  *last_.mutable_heading() = out.heading();
  out.clear_heading();

  // Rate of turn has opposite sign for Airmar.
  out.mutable_rate_turn()->set_rate(-state_.omega().z() + Normal(0.05));
  rate_turn_.send(&out);
  *last_.mutable_rate_turn() = out.rate_turn();
  out.clear_rate_turn();

  // Attitude only gets sent out at 1 Hz...
  if ((counter_ % int(1 / dt)) == 0) {
    const float roll = state_.euler().roll() + Normal(0.05);
    const float pitch = state_.euler().pitch() + Normal(0.05);
    out.mutable_attitude()->set_roll(CalcNew(last_.attitude().roll(), roll, kT1Hz));
    out.mutable_attitude()->set_pitch(
        CalcNew(last_.attitude().pitch(), pitch, kT1Hz));
    // TODO(james): Figure out why the airmar doesn't populate the yaw field
    attitude_.send(&out);
    *last_.mutable_attitude() = out.attitude();
    out.clear_attitude();
  }

  double x = state_.pos().x() + Normal(.5);
  double y = state_.pos().y() + Normal(.5);
  double lat = util::ToRad(y / lat_scale_. + lat_);
  double lon = util::ToRad(x / lon_scale_. + lon_);
  out.mutable_pos_rapid_update()->set_lon(lon);
  out.mutable_pos_rapid_update()->set_lat(lat);
  pos_rapid_update_.send(&out);
  *last_.mutable_pos_rapid_update() = out.pos_rapid_update();
  out.clear_pos_rapid_update();

  float vx = state_.vel().x();
  float vy = state_.vel().y();
  out.mutable_cog_rapid_update()->set_sog(std::sqrt(vx * vx + vy * vy) +
                                          Normal(0.1));
  out.mutable_cog_rapid_update()->set_cog(M_PI / 2. - std::atan2(vy, vx) +
                                          Normal(0.05));
  cog_rapid_update_.send(&out);
  *last_.mutable_cog_rapid_update() = out.cog_rapid_update();
  out.clear_cog_rapid_update();

  out.mutable_wind_data()->set_wind_speed(
      std::sqrt(wind_.x() * wind_.x() + wind_.y() * wind_.y()) +
      Normal(0.1));
  // Invert x/y to properly spoof airmar wind from, rather than wind dir.
  // Also, handle North vs. East 0-reference
  double true_wind_angle = util::norm_angle(
      M_PI / 2. - std::atan2(-wind_.y(), -wind_.x()) + Normal(0.05));
  out.mutable_wind_data()->set_wind_angle(true_wind_angle);
  out.mutable_wind_data()->set_reference(msg::can::WindData_WIND_REFERENCE_TRUE_NORTH_REF);
  wind_data_.send(&out);
  // And now do apparent wind...
  double wx = wind_.x() - vx;
  double wy = wind_.y() - vy;
  // Add M_PI to handle to vs. from wind dir.
  double alphaw =
      -util::norm_angle(std::atan2(-wy, -wx) - state_.euler().yaw());
  out.mutable_wind_data()->set_wind_speed(std::sqrt(wx * wx + wy * wy));
  out.mutable_wind_data()->set_wind_angle(alphaw);
  out.mutable_wind_data()->set_reference(msg::can::WindData_WIND_REFERENCE_APPARENT);
  wind_data_.send(&out);
  // TODO(james): Store both apparent and true wind...
  *last_.mutable_wind_data() = out.wind_data();
  out.clear_wind_data();

  ++counter_;
}

} // namespace sim
} // namespace sailbot
