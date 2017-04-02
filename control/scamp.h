#pragma once

#include "util/node.h"
#include "control/actuator_cmd.pb.h"

namespace sailbot {

class SCAMP : public Node {
 public:
  SCAMP();

 private:
  void Iterate() override;
  static int SBUSToRaw(int sbus) {
    return ((sbus - 1023) * 90) / 1000;
  };

  ProtoQueue<msg::InternalBoatState> state_queue_;
  msg::InternalBoatState *state_msg_;
  std::mutex state_msg_mut_;
  ProtoQueue<msg::can::CANMaster> pwm_queue_;
  msg::can::CANMaster *pwm_msg_;
  std::atomic<int> raw_winch_{90};
  std::atomic<int> raw_rudder_{90};;
  std::atomic<bool> is_manual_mode_{false};
  std::atomic<bool> is_connected_{true};
};  // class SCAMP

}  // namespace sailbot
