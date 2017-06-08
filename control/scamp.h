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

  float WinchPotToAngle(float pot_val);

  enum Actuator {
    RUDDER = 0,
    WINCH = 1,
  };

  int GetMode(Actuator a) { return a == RUDDER ? rudder_mode_ : winch_mode_; }

  bool IsDisabled(Actuator a) {
    return GetMode(a) == msg::ControlMode_MODE_DISABLE;
  }
  bool IsAuto(Actuator a) { return GetMode(a) == msg::ControlMode_MODE_AUTO; }
  bool IsManualRC(Actuator a) {
    return GetMode(a) == msg::ControlMode_MODE_MANUAL_RC;
  }
  bool IsManualWiFi(Actuator a) {
    return GetMode(a) == msg::ControlMode_MODE_MANUAL_WIFI;
  }
  bool IsFilteredRC(Actuator a) {
    return GetMode(a) == msg::ControlMode_MODE_FILTERED_RC;
  }

  ProtoQueue<msg::InternalBoatState> state_queue_;
  msg::InternalBoatState *state_msg_;
  std::mutex state_msg_mut_;
  ProtoQueue<msg::can::CANMaster> pwm_queue_;
  msg::can::CANMaster *pwm_msg_;
  ProtoQueue<msg::ZeroingConstants> consts_queue_;
  msg::ZeroingConstants *consts_msg_;
  std::atomic<int> raw_winch_{90};
  std::atomic<int> volts_winch_{90};
  std::atomic<int> raw_rudder_{90};;
  std::atomic<int> rudder_mode_{msg::ControlMode_MODE_DISABLE};
  std::atomic<int> winch_mode_{msg::ControlMode_MODE_DISABLE};
  std::atomic<bool> is_connected_{true};
};  // class SCAMP

}  // namespace sailbot
