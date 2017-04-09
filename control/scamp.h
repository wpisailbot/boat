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
    return ((sbus - 1023.) * 90.) / 1000. + 90.;
  };

  void SetRawFromSailCmd(float volts);
  void SetRawFromRudderCmd(const msg::RudderCmd &cmd);

  bool IsDisabled() { return control_mode_ == msg::ControlMode_MODE_DISABLE; }
  bool IsAuto() { return control_mode_ == msg::ControlMode_MODE_AUTO; }
  bool IsManualRC() { return control_mode_ == msg::ControlMode_MODE_MANUAL_RC; }
  bool IsManualWiFi() { return control_mode_ == msg::ControlMode_MODE_MANUAL_WIFI; }

  ProtoQueue<msg::InternalBoatState> state_queue_;
  msg::InternalBoatState *state_msg_;
  std::mutex state_msg_mut_;
  ProtoQueue<msg::can::CANMaster> pwm_queue_;
  msg::can::CANMaster *pwm_msg_;
  std::atomic<int> raw_winch_{90};
  std::atomic<int> raw_rudder_{90};;
  std::atomic<int> control_mode_{msg::ControlMode_MODE_DISABLE};
  std::atomic<bool> is_connected_{true};
};  // class SCAMP

}  // namespace sailbot
