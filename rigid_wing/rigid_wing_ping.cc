#include "rigid_wing.h"
#include "gflags.h"
#include "ui/server.h"

DEFINE_int32(rwstate, 0, "State to send to rigid wing");
DEFINE_double(rwheel, M_PI / 2, "Heel angle to rigid wing");
DEFINE_double(rwmax_heel, 0, "Max heel angle to rigid wing");
DEFINE_int32(rwservo, 0, "Goal servo pos to send rigid wing");

class Pinger : public sailbot::Node {
 public:
  Pinger() : Node(1), cmd_queue_("rigid_wing_cmd", true) {}
  void Iterate() override {
    return;
    sailbot::msg::RigidWingCmd cmd;
    cmd.set_state((sailbot::msg::RigidWingCmd_WingState)FLAGS_rwstate);
    cmd.set_heel(FLAGS_rwheel);
    cmd.set_max_heel(FLAGS_rwmax_heel);
    cmd.set_servo_pos(FLAGS_rwservo);
    cmd_queue_.send(&cmd);
  }

  sailbot::ProtoQueue<sailbot::msg::RigidWingCmd> cmd_queue_;
};

int main(int argc, char *argv[]) {
  sailbot::Queue::set_testing(true);
  sailbot::util::Init(argc, argv);
  sailbot::RigidWing wing;
  sailbot::WebSocketServer server;
  Pinger pinger;
  std::thread t(&Pinger::Run, &pinger);
  std::thread tserver(&sailbot::WebSocketServer::Run, &server);
  wing.Run();
  t.join();
  tserver.join();
}
