#include "util/node.h"
#include "util/msg.pb.h"
#include "ipc/queue.hpp"
#include <stdio.h>

namespace sailbot {

class Pong : public Node {
 public:
  Pong() : Node(0.1), queue_("pong", true) {
    msg_ = AllocateMessage<msg::PongMsg>();
    RegisterHandler<msg::PingMsg>("ping", [this](const msg::PingMsg &msg) {
      msg_->set_b(2);
      queue_.send(msg_);
      LOG(INFO) << msg.a();
    });
  }

 protected:
  virtual void Iterate() {}

 private:
  ProtoQueue<msg::PongMsg> queue_;
  msg::PongMsg *msg_;
};

}  // sailbot

int main() {
  sailbot::Pong pong;
  pong.Run();
}
