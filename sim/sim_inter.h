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
    impl_.set_wind(
        Vector3d(speed * std::cos(src_dir), speed * std::sin(src_dir), 0));
  }

 private:
  void Iterate() override;

  void ProcessSail(const msg::SailCmd& cmd);
  void ProcessRudder(const msg::RudderCmd& cmd);

  static constexpr double dt = 0.001;
  SimulatorSaoud2013 impl_;
  double sdot_, rdot_;

  ProtoQueue<msg::BoatState> state_queue_;
  msg::BoatState* state_msg_;
};

}  // sim
}  // sailbot
