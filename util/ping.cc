#include "util/node.h"
#include "util/msg.pb.h"
#include "ipc/queue.hpp"
#include "glog/logging.h"

namespace sailbot {

class Ping : public Node {
 public:
  Ping() : Node(0.1), queue_("ping"), msg_(AllocateMessage<PingMsg>()) {
    RegisterHandler<PongMsg>("pong",
                             [](const PongMsg &msg) { LOG(INFO) << msg.b(); });
  }

 protected:
  virtual void Iterate() {
    msg_->set_a(msg_->a() + 0.001);
    queue_.send(msg_);
  }

 private:
  ProtoQueue<PingMsg> queue_;
  PingMsg *msg_;
};

}  // sailbot

int main() {
  sailbot::Ping ping;
  ping.Run();
}
