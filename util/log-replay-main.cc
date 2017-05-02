#include "log-replay.h"
#include "ui/server.h"
#include "sim/csv_logging.h"
#include <iomanip>
#include <gflags/gflags.h>

DEFINE_string(csv_file, "sim/python/basic_replay_data.csv", "File to save CSV data to");
DEFINE_int64(start_time, 50000, "Time of day at which to slow down");

namespace sailbot {
class Pong : public Node {
 public:
  Pong(CsvLogger* csv) : Node(0.1), queue_("pong", true), csv_(csv) {
    msg_ = AllocateMessage<msg::PongMsg>();
    RegisterHandler<msg::PingMsg>("ping", [this](const msg::PingMsg &msg) {
      msg_->set_b(2);
      queue_.send(msg_);
      LOG(INFO) << msg.a();
    });

    RegisterHandler<msg::can::CANMaster>(
        "can65282", [this](const msg::can::CANMaster &msg) {
          if (msg.has_pwm_write())
            LOG(INFO) << "Winch: " << (msg.pwm_write().winch() - 90.) * 12. / 90.;
        });

    RegisterHandler<msg::can::CANMaster>(
        "can126992", [this](const msg::can::CANMaster &msg) {
          if (msg.has_sys_tme())
            LOG(INFO) << "Sec: " << msg.sys_tme().time();
            time_ = msg.sys_tme().time() * 1e-4;
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
    if (time_ > FLAGS_start_time && time_ < 184000) {
      // https://photos.google.com/share/AF1QipPKE4vvzHhIxVOMlYVmTDNU-MZcG-CIdRSTCXDK-YPGFDn8JFm3YVoFXDOs0D3-oQ/photo/AF1QipPJ1aV6olSGbYjYVkOZodEsbAkSvNt0uMlDQ7qq?key=VWlrb2ZXU3lOZGU5bDJ6QkFiUFhuMkFoWmMzWHpR @ ~9:00
      // logs/2017-04-11/logfile-1486480636
      // https://photos.google.com/share/AF1QipPKE4vvzHhIxVOMlYVmTDNU-MZcG-CIdRSTCXDK-YPGFDn8JFm3YVoFXDOs0D3-oQ/photo/AF1QipMskZdUYDQ6n64tLzBnukosbA1qqopdpdANdEmy?key=VWlrb2ZXU3lOZGU5bDJ6QkFiUFhuMkFoWmMzWHpR
      csv_->Start();
      usleep(0.2 * 1e5);
    }
  }

 private:
  ProtoQueue<msg::PongMsg> queue_;
  msg::PongMsg *msg_;
  std::atomic<int> time_{0};
  CsvLogger *csv_;
};
}  // sailbot

int main(int argc, char *argv[]) {
  sailbot::Queue::set_testing(true);
  sailbot::util::Init(argc, argv);

  sailbot::LogReplay replay;
  sailbot::WebSocketServer server;
  sailbot::CsvLogger csv({{"wind.x", "Wind X"},
                          {"wind.y", "Wind Y"},
                          {"boat_state.internal.sail", "Sail"},
                          {"boat_state.internal.rudder", "Rudder"},
                          {"boat_state.euler.yaw", "Yaw"},
                          {"boat_state.euler.roll", "Heel"},
                          {"boat_state.pos.x", "Longitude"},
                          {"boat_state.pos.y", "Latitude"},
                          {"boat_state.vel.x", "Vel X"},
                          {"boat_state.vel.y", "Vel Y"},
                          {"true_wind.x", "True Wind X"},
                          {"true_wind.y", "True Wind Y"}}, FLAGS_csv_file, 0.1);
  csv.Stop();
  sailbot::Pong pong(&csv);

  std::thread rthread(&sailbot::LogReplay::Run, &replay);
  std::thread pthread(&sailbot::Pong::Run, &pong);
  std::thread csvthread(&sailbot::CsvLogger::Run, &csv);

  server.Run();

  rthread.join();
  pthread.join();
  csvthread.join();
}
