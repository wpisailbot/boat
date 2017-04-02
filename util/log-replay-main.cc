#include "log-replay.h"
#include "ui/server.h"
#include <iomanip>

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

    RegisterHandler<msg::can::CANMaster>(
        "can126992", [this](const msg::can::CANMaster &msg) {
          if (msg.has_sys_tme())
            LOG(INFO) << "Sec: " << msg.sys_tme().time();
        });

    RegisterHandler<msg::can::CANMaster>(
        "can127250", [this](const msg::can::CANMaster &msg) {
          if (msg.has_heading())
            LOG(INFO) << "Heading: " << msg.heading().DebugString();
        });

    RegisterHandler<msg::can::CANMaster>(
        "can127257", [this](const msg::can::CANMaster &msg) {
          if (msg.has_attitude())
            LOG(INFO) << "Attitude: " << msg.attitude().DebugString();
        });

    RegisterHandler<msg::can::CANMaster>(
        "can130306", [this](const msg::can::CANMaster &msg) {
          if (msg.has_wind_data())
            LOG(INFO) << "Wind: " << msg.wind_data().DebugString();
        });

    RegisterHandler<msg::can::CANMaster>(
        "can129025", [this](const msg::can::CANMaster &msg) {
          if (msg.has_pos_rapid_update())
            LOG(INFO) << "Lat,Lon: " << std::setprecision(15)
                      << msg.pos_rapid_update().lat() << ", "
                      << msg.pos_rapid_update().lon();
        });
    RegisterHandler<msg::can::CANMaster>(
        "can129539", [this](const msg::can::CANMaster &msg) {
          if (msg.has_gnss_dop())
            LOG(INFO) << "hdop,vdop,tdop: " << msg.gnss_dop().hdop() << ", "
                      << msg.gnss_dop().vdop() << ", " << msg.gnss_dop().tdop();
        });
    RegisterHandler<msg::BoatState>(
        "boat_state", [this](const msg::BoatState &msg) {
            LOG(INFO) << "Boat State: " << msg.DebugString();
        });
  }

 protected:
  virtual void Iterate() {
    usleep(1e5);
  }

 private:
  ProtoQueue<msg::PongMsg> queue_;
  msg::PongMsg *msg_;
};
}  // sailbot

int main(int argc, char *argv[]) {
//  sailbot::Queue::set_testing(true);
  sailbot::util::Init(argc, argv);

  sailbot::LogReplay replay;
  sailbot::Pong pong;
  sailbot::WebSocketServer server;

  std::thread rthread(&sailbot::LogReplay::Run, &replay);
  std::thread pthread(&sailbot::Pong::Run, &pong);

  server.Run();

  rthread.join();
  pthread.join();
}
