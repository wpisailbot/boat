#include "util/node.h"
#include "util/msg.pb.h"
#include "ipc/queue.hpp"
#include <stdio.h>

namespace sailbot {

class Pong : public Node {
 public:
  Pong() : Node(0.1), queue_("pong") {
    msg_ = AllocateMessage<PongMsg>();
    RegisterHandler<PingMsg>("ping", [this](const PingMsg &msg) {
      msg_->set_b(2);
      queue_.send(msg_);
      LOG(INFO) << msg.a();
    });
  }

 protected:
  virtual void Iterate() {}

 private:
  ProtoQueue<PongMsg> queue_;
  PongMsg *msg_;
};

}  // sailbot

int main() {
  sailbot::Pong pong;
  pong.Run();
}
