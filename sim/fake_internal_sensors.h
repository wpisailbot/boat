#pragma once

#include "util/node.h"
#include "control/actuator_cmd.pb.h"

namespace sailbot {
namespace sim {

/**
 * This class is for use with the simulator in faking the position sensing for
 * the various actuators. Currently, just passes the truth values straight
 * through.
 */
class FakeInternalSensors : public Node {
 public:
  FakeInternalSensors();
 private:
  void Iterate() override {}

  ProtoQueue<msg::InternalBoatState> output_queue_;
};

}  // namespace sim
}  // namespace sailbot
