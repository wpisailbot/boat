#pragma once

#include "util/node.h"
#include "control/actuator_cmd.pb.h"

namespace sailbot {

class ArduinoMotors : public Node {
 public:
   ArduinoMotors(const char *port);

 private:
  const int kBaudRate = 115200;

  void Init();
  void Iterate() override;

  const char * const port_name_;
  ProtoQueue<msg::InternalBoatState> state_queue_;
  std::mutex state_msg_mut_;
  msg::InternalBoatState *state_msg_;
  int fd_ = -1;
  std::atomic<int> raw_winch_{90};
  std::atomic<int> raw_rudder_{90};;
};  // class ArduinoMotors

}  // namespace sailbot
