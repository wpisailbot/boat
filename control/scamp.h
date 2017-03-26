#pragma once

#include "util/node.h"
#include "control/actuator_cmd.pb.h"

namespace sailbot {

class SCAMP : public Node {
 public:
  SCAMP();

 private:
  void Iterate() override;

  ProtoQueue<msg::InternalBoatState> state_queue_;
  msg::InternalBoatState *state_msg_;
  std::mutex state_msg_mut_;
  ProtoQueue<msg::can::CANMaster> pwm_queue_;
  msg::can::CANMaster *pwm_msg_;
  std::atomic<int> raw_winch_{90};
  std::atomic<int> raw_rudder_{90};;
};  // class SCAMP

}  // namespace sailbot
