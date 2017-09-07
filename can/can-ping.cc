#include "util/msg.pb.h"
#include "util/node.h"

namespace sailbot {

class CanPing : public Node {
 public:
   CanPing()
       : Node(0.5), msg_(AllocateMessage<msg::can::CANMaster>()),
         queue_("can65284", true) {
     msg_->set_outgoing(true);
     RegisterHandler<msg::can::CANMaster>(
         "can65283", [this](const msg::can::CANMaster &msg) {
           msg_->mutable_debug_scamp2()->set_data1(123);
           msg_->mutable_debug_scamp2()->set_data2(msg.debug_scamp1().data1());
           queue_.send(msg_);
         });
   }

 private:
   void Iterate() override {}
  msg::can::CANMaster *msg_;
  ProtoQueue<msg::can::CANMaster> queue_;
};
}  // namespace sailbot

int main(int argc, char *argv[]) {
  sailbot::util::SetCurrentThreadRealtimePriority(10);
  sailbot::util::Init(argc, argv);
  sailbot::CanPing ping;
  ping.Run();
}
