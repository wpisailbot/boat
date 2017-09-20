#pragma once
#include "util/node.h"
#include <functional>
#include "control/actuator_cmd.pb.h"
#include "sim_physics.h"
#include "fake_internal_sensors.h"
#include "fake_airmar.h"

namespace sailbot {
namespace sim {

class SimulatorNode : public Node {
 public:
  SimulatorNode(double reset_period);

  void Run() override;

  void set_wind(float src_dir, float speed /*m/s*/) {
    float x = speed * std::cos(src_dir);
    float y = speed * std::sin(src_dir);
    set_wind(x, y, 0);
  }
  void set_wind(float x /*m/s*/, float y /*m/s*/, float z /*m/s*/) {
    impl_->set_wind(Vector3d(x, y, z));

    wind_msg_->set_x(x);
    wind_msg_->set_y(y);
    wind_msg_->set_z(z);
    wind_queue_.send(wind_msg_);
  }

  //Eigen::Vector3d get_x() { return impl_->get_x(); }
  float get_x() { return state_msg_->pos().x(); }
  float get_y() { return impl_->get_x()(1, 0); }

 private:
  void Iterate() override;

  void ProcessSail(const msg::SailCmd& cmd);
  void ProcessRudder(const msg::RudderCmd& cmd);
  void ProcessBallast(const msg::BallastCmd& cmd);

  static constexpr double dt = 0.001;
  const double reset_period_;
  std::unique_ptr<TrivialDynamics> impl_;
  std::atomic<double> sdot_, rdot_, bdot_;
  bool started_{false};
  std::atomic<double> next_reset_{-1};

  ProtoQueue<msg::BoatState> state_queue_;
  msg::BoatState* state_msg_;
  std::mutex last_state_mutex_;
  msg::BoatState last_state_;
  ProtoQueue<msg::Vector3f> wind_queue_;
  msg::Vector3f* wind_msg_;

  FakeInternalSensors internal_sensors_;
  FakeAirmar fake_airmar_;
};

}  // sim
}  // sailbot
