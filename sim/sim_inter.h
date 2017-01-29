#pragma once
#include "util/node.h"
#include <functional>
#include "control/actuator_cmd.pb.h"
#include "sim_physics.h"

namespace sailbot {
namespace sim {

class SimulatorNode : public Node {
 public:
  SimulatorNode();

  void set_wind(float src_dir, float speed /*m/s*/) {
    float x = speed * std::cos(src_dir);
    float y = speed * std::sin(src_dir);
    impl_->set_wind(Vector3d(x, y, 0));
    wind_msg_->set_x(x);
    wind_msg_->set_y(y);
    wind_msg_->set_z(0);
    wind_queue_.send(wind_msg_);
  }

  //Eigen::Vector3d get_x() { return impl_->get_x(); }
  float get_x() { return state_msg_->pos().x(); }
  float get_y() { return impl_->get_x()(1, 0); }

 private:
  void Iterate() override;

  void ProcessSail(const msg::SailCmd& cmd);
  void ProcessRudder(const msg::RudderCmd& cmd);

  static constexpr double dt = 0.001;
  std::unique_ptr<TrivialDynamics> impl_;
  std::atomic<double> sdot_, rdot_;

  ProtoQueue<msg::BoatState> state_queue_;
  msg::BoatState* state_msg_;
  ProtoQueue<msg::Vector3f> wind_queue_;
  msg::Vector3f* wind_msg_;
};

}  // sim
}  // sailbot
