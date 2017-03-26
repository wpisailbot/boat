#pragma once
#include "util/node.h"
#include "sensor/sbus.pb.h"

namespace sailbot {

class ReadSBUS : public Node {
 public:
  ReadSBUS(const char *port)
      : Node(0.0), port_name_(port), msg_(AllocateMessage<msg::SBUS>()),
        queue_("sbus_value", true) {
    Init();
  }
  ~ReadSBUS() {
    close(fd);
  }
 private:
  const int kBaudRate = 100000;
  int fd;
  const char* port_name_;
  msg::SBUS* msg_;
  ProtoQueue<msg::SBUS> queue_;

  void Init();
  void Iterate() override;
};

}  // sailbot
