#include "rigid_wing.h"
#include "gflags.h"

DEFINE_int32(rwstate, 0, "State to send to rigid wing");
DEFINE_double(rwheel, M_PI / 2, "Heel angle to rigid wing");
DEFINE_double(rwmax_heel, 0, "Max heel angle to rigid wing");
DEFINE_double(rwaoa, 0, "Goal angle of attack to send rigid wing");

class Pinger : public sailbot::Node {
 public:
  Pinger() : Node(1), cmd_queue_("rigid_wing_cmd", true) {}
  void Iterate() override {
    sailbot::msg::RigidWingCmd cmd;
    cmd.set_state((sailbot::msg::RigidWingCmd_WingState)FLAGS_rwstate);
    cmd.set_heel(FLAGS_rwheel);
    cmd.set_max_heel(FLAGS_rwmax_heel);
    cmd.set_angle_of_attack(FLAGS_rwaoa);
    cmd_queue_.send(&cmd);
  }

  sailbot::ProtoQueue<sailbot::msg::RigidWingCmd> cmd_queue_;
};

int main(int argc, char *argv[]) {
  sailbot::Queue::set_testing(true);
  sailbot::util::Init(argc, argv);
  sailbot::RigidWing wing;
  Pinger pinger;
  std::thread t(&Pinger::Run, &pinger);
  wing.Run();
  t.join();
}
