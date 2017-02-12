#pragma once

#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include "can/can.pb.h"

namespace sailbot {
namespace sim {

/**
 * This class is for use with the simulator in faking the Airmar 220WX
 */
class FakeAirmar : public Node {
 public:
  FakeAirmar();
 private:
  static constexpr float dt = 0.1;

  void Iterate() override;

  // For locking access to all the state.
  std::mutex state_msg_mutex_;
  // Storage for boat state.
  msg::BoatState state_;
  msg::Vector3f wind_;
  ProtoQueue<msg::can::CANMaster> heading_;
  ProtoQueue<msg::can::CANMaster> rate_turn_;
  ProtoQueue<msg::can::CANMaster> attitude_;
  ProtoQueue<msg::can::CANMaster> pos_rapid_update_;
  ProtoQueue<msg::can::CANMaster> cog_rapid_update_;
  ProtoQueue<msg::can::CANMaster> wind_data_;

  int counter_; // For keeping track of the number of iterations.
};

}  // namespace sim
}  // namespace sailbot
