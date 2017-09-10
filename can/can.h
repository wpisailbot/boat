#pragma once

#include <map>
#include <memory>
#include "canboat-pgn.h"
#include "can/can.pb.h"
#include "util/node.h"
#include <linux/can.h>

namespace sailbot {
namespace can {

namespace testing { class CanTest; }

class CanNode : public Node {
  struct CANMessage {
    const Pgn* pgn;
    bool is_long_;
    size_t len_;
    uint8_t base_; // The prefix for multi-packet messages
    std::unique_ptr<uint8_t[]> data_;
    std::unique_ptr<ProtoQueue<msg::can::CANMaster>> queue_;
    msg::can::CANMaster* store_msg_;

    CANMessage(const Pgn *p, const char *queue_name, msg::can::CANMaster* store)
        : pgn(p), is_long_(p->size > 8), len_(p->size), base_(0),
          data_(std::make_unique<uint8_t[]>(((int)(len_ / 8) + 1) * 8)),
          queue_(std::make_unique<ProtoQueue<msg::can::CANMaster>>(queue_name,
                                                                   true)),
          store_msg_(store) {}
    CANMessage(CANMessage &&) = default;
    CANMessage() {}
  };
 public:
  CanNode();
  ~CanNode() { close(s_); }
  void Iterate();
 private:
  int s_;
  // Storage for the raw message contents.
  std::map<uint32_t, CANMessage> msgs_;
  static void DecodeAndSend(const CANMessage *msg);
  // Extracts the Field f from CAN data data.
  // start_bit is the bit in data where the field starts.
  // maxval is set to the maximum absolute value of the returned value.
  // Returns the raw value (does NOT scale for resolution).
  static int64_t ExtractNumberField(const Field *f, const uint8_t *data,
                                    size_t start_bit, int64_t *maxval);

  void SendMessage(const msg::can::CANMaster &msg, const int pgn);
  // Returns true if succeeded at constructing message.
  static bool ConstructMessage(const msg::can::CANMaster &msg,
                               const CANMessage *msg_info, can_frame *frame);

  friend sailbot::can::testing::CanTest;
};

}  // namespace sailbot
}  // namespace can
