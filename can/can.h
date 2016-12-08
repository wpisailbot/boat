#pragma once

#include <map>
#include <memory>
#include "canboat-pgn.h"
#include "can/can.pb.h"
#include "util/node.h"

namespace sailbot {
namespace can {

class CanNode : public Node {
  struct CANMessage {
    const Pgn* pgn;
    bool is_long_;
    size_t len_;
    uint8_t base_;
    std::unique_ptr<uint8_t[]> data_;
    std::unique_ptr<ProtoQueue<msg::can::CANMaster>> queue_;
    msg::can::CANMaster* store_msg_;

    CANMessage(const Pgn *p, const char *queue_name, msg::can::CANMaster* store)
        : pgn(p), is_long_(p->size > 8), len_(p->size), base_(0),
          data_(std::make_unique<uint8_t[]>(len_)),
          queue_(std::make_unique<ProtoQueue<msg::can::CANMaster>>(queue_name,
                                                                   true)),
          store_msg_(store) {}
    CANMessage() {}
  };
 public:
  CanNode();
  void Iterate();
 private:
  int s_;
  // Storage for the raw message contents.
  std::map<uint32_t, CANMessage> msgs_;
  void DecodeAndSend(const CANMessage* msg);
  double ExtractNumberField(const Field * f, const uint8_t *data, size_t start_bit);
};

}  // namespace sailbot
}  // namespace can
