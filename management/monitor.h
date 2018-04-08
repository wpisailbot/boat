#pragma once

#include "util/node.h"
#include "can/can.pb.h"

namespace sailbot {

class Monitor : public Node {
 public:
  Monitor();
 private:
  const int kRigidTimeout = 5; // Timeout on rigid wing; seconds
  void Iterate() override;

  std::atomic<int> rigid_mode_{msg::ControlMode_MODE_DISABLE};
  std::atomic<int> rudder_mode_{msg::ControlMode_MODE_DISABLE};
  std::atomic<int> winch_mode_{msg::ControlMode_MODE_DISABLE};
  std::atomic<int> ballast_mode_{msg::ControlMode_MODE_DISABLE};
  std::atomic<int> tacker_mode_{msg::ControlMode_TACKER_NONE};

  std::atomic<bool> ui_connection_{false};
  std::atomic<bool> rigid_connection_{false};

  std::mutex last_rigid_conn_mutex_;
  util::monotonic_clock::time_point last_rigid_conn_;

  std::mutex state_mutex_;
  msg::can::CANMaster *state_msg_;
  msg::can::CANMaster *connection_msg_;
  ProtoQueue<msg::can::CANMaster> state_queue_;
  ProtoQueue<msg::can::CANMaster> connection_queue_;
}; // class Monitor

}  // namespace sailbot
