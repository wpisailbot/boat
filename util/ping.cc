#include "util/node.h"
#include "util/msg.pb.h"
#include "ipc/queue.hpp"
#include "glog/logging.h"
#include <map>

namespace sailbot {

class Ping : public Node {
 public:
  Ping()
      : Node(0.1), queue_("ping", true), msg_(AllocateMessage<msg::PingMsg>()) {
    RegisterHandler<msg::PongMsg>(
        "pong", [](const msg::PongMsg &msg) { LOG(INFO) << msg.b(); });
  }

 protected:
  virtual void Iterate() {
    msg_->set_a(msg_->a() + 0.001);
    queue_.send(msg_);
  }

 private:
  ProtoQueue<msg::PingMsg> queue_;
  msg::PingMsg *msg_;
};

}  // sailbot

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);
  sailbot::Ping ping;
  ping.Run();
}
