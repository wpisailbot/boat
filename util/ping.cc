#include "util/node.h" // For the class Node
#include "util/msg.pb.h" // For the PongMsg and PingMsg protobufs
#include "ipc/queue.hpp" // For the ProtoQueue sender
#include "glog/logging.h" // To do logging/printing

// All sailbot classes are in namespace sailbot
namespace sailbot {

class Ping : public Node {
 public:
  Ping()
      : Node(0.001), queue_("ping", true), msg_(AllocateMessage<msg::PingMsg>()) {
    RegisterHandler<msg::PongMsg>(
        "pong", [](const msg::PongMsg &msg) { LOG(INFO) << msg.b(); });
  }

 private:
  void Iterate() override {
    msg_->set_a(msg_->a() + 0.001);
    queue_.send(msg_);
  }

  ProtoQueue<msg::PingMsg> queue_;
  msg::PingMsg *msg_;
};

}  // namespace sailbot

int main(int argc, char *argv[]) {
  // All programs should call Init()
  sailbot::util::Init(argc, argv);
  // Create and run our Ping node, using the Run function inherited  from Node
  sailbot::Ping ping;
  ping.Run();
}
