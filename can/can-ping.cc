#include "can.h"
#include <thread>

namespace sailbot {

// Just sends out a rate-of-turn message regularly.
class CanPing : public Node {
 public:
   CanPing()
       : Node(0.1), queue_("can127251", true),
         msg_(AllocateMessage<msg::can::CANMaster>()) {
     msg_->set_outgoing(true);
     RegisterHandler<msg::can::CANMaster>("can127251",
                                          [](const msg::can::CANMaster &msg) {
       LOG(INFO) << "Got rate turn message";
     });
   }

 private:
  void Iterate() override {
    LOG(INFO) << "Sending out request to send";
    msg_->mutable_rate_turn()->set_sid(b++);
    msg_->mutable_rate_turn()->set_rate(a += 100*3e-9);
   // queue_.send(msg_);
  }
  ProtoQueue<msg::can::CANMaster> queue_;
  msg::can::CANMaster *msg_;
  uint8_t b = 0;
  double a = 0;
};
}  // namespace sailbot

int main(int argc, char *argv[]) {
  sailbot::Queue::set_testing(true);
  sailbot::util::Init(argc, argv);
  sailbot::can::CanNode can;
  sailbot::CanPing ping;
  std::thread t(&sailbot::can::CanNode::Run, &can);
  ping.Run();
  t.join();
}
