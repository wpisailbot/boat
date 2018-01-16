#pragma once

#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include "control/util.h"
#include "can/can.pb.h"

namespace sailbot {
namespace sim {

/**
 * This class is for use with the simulator in faking the Airmar 220WX
 */
class FakeAirmar : public Node {
 public:
  FakeAirmar();

  void set_gps_zero(double lat, double lon) {
    lat_ = lat;
    lon_ = lon;
    const double eps = 1e-6;
    double latscale, lonscale;
    util::GPSLatLonScale(lat, &latscale, &lonscale);
    lat_scale_ = latscale;
    lon_scale_ = lonscale;
  }
 private:
  static constexpr float dt = 0.1;
  // Pole for assuming exponential decay to the truth
  // values, assuming that the airmar orientation all
  // has noticeable delay. Assumes units of seconds,
  // to calculate for discrete intervals, use
  // e^(dt * kTContinuous)
  static constexpr float kTContinuous = -2.1;

  void Iterate() override;

  // For locking access to all the state.
  std::mutex state_msg_mutex_;
  msg::can::CANMaster last_;
  // Storage for boat state.
  msg::BoatState state_;
  msg::Vector3f wind_;
  ProtoQueue<msg::can::CANMaster> heading_;
  ProtoQueue<msg::can::CANMaster> rate_turn_;
  ProtoQueue<msg::can::CANMaster> attitude_;
  ProtoQueue<msg::can::CANMaster> pos_rapid_update_;
  ProtoQueue<msg::can::CANMaster> cog_rapid_update_;
  ProtoQueue<msg::can::CANMaster> wind_data_;
  // Radians of lat/lon
  std::atomic<double> lat_{38.9816688 * M_PI / 180.},
      lon_{-76.47591338 * M_PI / 180.};
  // Number of meters per radian latitude.
  std::atomic<double> lat_scale_{111015. / M_PI * 180.},
      lon_scale_{86647. / M_PI * 180.};

  int counter_; // For keeping track of the number of iterations.
};

}  // namespace sim
}  // namespace sailbot
