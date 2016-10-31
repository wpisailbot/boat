#pragma once
#include "util/node.h"
#include <functional>
#include "control/actuator_cmd.pb.h"
#include "sim_physics.h"

namespace sailbot {
namespace sim {

class SimulatorNode : public Node {
 public:
  SimulatorNode()
      : Node(.001),
        sdot_(0),
        rdot_(0),
        state_queue_("boat_state"),
        state_msg_(AllocateMessage<msg::BoatState>()) {
    RegisterHandler<msg::SailCmd>(
        "sail_cmd",
        std::bind(&SimulatorNode::ProcessSail, this, std::placeholders::_1));
    RegisterHandler<msg::RudderCmd>(
        "rudder_cmd",
        std::bind(&SimulatorNode::ProcessRudder, this, std::placeholders::_1));
  }
 private:
  void Iterate();

  void ProcessSail(const msg::SailCmd& cmd);
  void ProcessRudder(const msg::RudderCmd& cmd);
  SimulatorSaoud2013 impl_;
  double sdot_, rdot_;

  ProtoQueue<msg::BoatState> state_queue_;
  msg::BoatState* state_msg_;
};

}  // sim
}  // sailbot
