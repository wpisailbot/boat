#include "log-replay.h"
#include "ui/server.h"
#include "sim/csv_logging.h"
#include <iomanip>
#include <gflags/gflags.h>

DEFINE_string(csv_file, "/tmp/basic_replay_data.csv", "File to save CSV data to");
DEFINE_int64(start_time, 50000, "Time of day at which to slow down");
DEFINE_double(speedup, 1, "Factor by which to speed up the replay once we hit start_time");

namespace sailbot {
class Pong : public Node {
 public:
  Pong(CsvLogger* csv) : Node(0.1), queue_("pong", true), csv_(csv) {
    msg_ = AllocateMessage<msg::PongMsg>();

    RegisterHandler<msg::can::CANMaster>(
        "can65282", [this](const msg::can::CANMaster &msg) {
          if (msg.has_pwm_write())
            VLOG(1) << "Winch: " << (msg.pwm_write().winch() - 90.) * 12. / 90.;
        });

    RegisterHandler<msg::can::CANMaster>(
        "can126992", [this](const msg::can::CANMaster &msg) {
          if (msg.has_sys_time())
            VLOG(1) << "Sec: " << msg.sys_time().time();
            time_ = msg.sys_time().time() * 1e-4;
        });

    RegisterHandler<msg::can::CANMaster>(
        "can127250", [this](const msg::can::CANMaster &msg) {
          if (msg.has_heading())
            VLOG(1) << "Heading: " << msg.heading().DebugString();
        });

    RegisterHandler<msg::can::CANMaster>(
        "can127257", [this](const msg::can::CANMaster &msg) {
          if (msg.has_attitude())
            VLOG(1) << "Attitude: " << msg.attitude().DebugString();
        });

    RegisterHandler<msg::can::CANMaster>(
        "can130306", [this](const msg::can::CANMaster &msg) {
          if (msg.has_wind_data())
            VLOG(1) << "Wind: " << msg.wind_data().DebugString();
          return;
          if (msg.wind_data().reference() == msg::can::WindData::APPARENT) {
            std::cout << msg.wind_data().wind_speed() << ","
                      << msg.wind_data().wind_angle() << "\n";
          }
        });

    RegisterHandler<msg::can::CANMaster>(
        "can129025", [this](const msg::can::CANMaster &msg) {
          if (msg.has_pos_rapid_update())
            VLOG(1) << "Lat,Lon: " << std::setprecision(15)
                    << msg.pos_rapid_update().lat() << ", "
                    << msg.pos_rapid_update().lon();
        });
    RegisterHandler<msg::can::CANMaster>(
        "can129539", [this](const msg::can::CANMaster &msg) {
          if (msg.has_gnss_dop())
            VLOG(1) << "hdop,vdop,tdop: " << msg.gnss_dop().hdop() << ", "
                      << msg.gnss_dop().vdop() << ", " << msg.gnss_dop().tdop();
        });
    RegisterHandler<msg::BoatState>(
        "boat_state", [this](const msg::BoatState &msg) {
            VLOG(1) << "Boat State: " << msg.DebugString();
        });
    RegisterHandler<msg::can::CANMaster>(
        "can65283", [this](const msg::can::CANMaster &msg) {
          uint32_t d1 = msg.debug_scamp1().data1();
          uint32_t d2 = msg.debug_scamp1().data2();
          int tms = d1 & 0xFFFF;
          int enct = (int16_t)(d1 >> 16);
          float airmar = 1e-7 / 32. * (int32_t)((d2 & 0xFFFF) << 16);
          int encairmar  = (int16_t)(d2 >> 16);
          if (print_air_) {
            std::cout << tms << "," << enct << "," << airmar << "\n";
          }
        });
    RegisterHandler<msg::PingMsg>("ping", [this](const msg::PingMsg &msg) {
      LOG(INFO) << "a: " << msg.a();
      if (std::abs(msg.a() - 1.5) < 1e-5) {
        LOG(INFO) << "Starting rate of turn";
        print_air_ = true;
      } else if (std::abs(msg.a() - 1.59) < 1e-5) {
        print_air_ = false;
        LOG(INFO) << "Ending rate of turn";
      }
    });
  }

 protected:
  virtual void Iterate() {
    if (time_ > FLAGS_start_time && time_ < 184000) {
      // https://photos.google.com/share/AF1QipPKE4vvzHhIxVOMlYVmTDNU-MZcG-CIdRSTCXDK-YPGFDn8JFm3YVoFXDOs0D3-oQ/photo/AF1QipPJ1aV6olSGbYjYVkOZodEsbAkSvNt0uMlDQ7qq?key=VWlrb2ZXU3lOZGU5bDJ6QkFiUFhuMkFoWmMzWHpR @ ~9:00
      // logs/2017-04-11/logfile-1486480636
      // https://photos.google.com/share/AF1QipPKE4vvzHhIxVOMlYVmTDNU-MZcG-CIdRSTCXDK-YPGFDn8JFm3YVoFXDOs0D3-oQ/photo/AF1QipMskZdUYDQ6n64tLzBnukosbA1qqopdpdANdEmy?key=VWlrb2ZXU3lOZGU5bDJ6QkFiUFhuMkFoWmMzWHpR
      csv_->Start();
      usleep(1e5 / FLAGS_speedup);
    }
  }

 private:
  ProtoQueue<msg::PongMsg> queue_;
  msg::PongMsg *msg_;
  std::atomic<int> time_{0};
  bool print_air_{false};
  CsvLogger *csv_;
};
}  // sailbot

int main(int argc, char *argv[]) {
  std::cout << "#speed,angle\n";
  sailbot::Queue::set_testing(true);
  sailbot::util::Init(argc, argv);

  sailbot::LogReplay replay({{}}, false);
  sailbot::WebSocketServer server;
#if 1
  sailbot::CsvLogger csv({{"wind.x", "Wind X"}, // 0
                          {"wind.y", "Wind Y"}, // 1
                          {"boat_state.internal.sail", "Sail"}, // 2
                          {"boat_state.internal.rudder", "Rudder"}, // 3
                          {"boat_state.euler.yaw", "Yaw"}, // 4
                          {"boat_state.euler.roll", "Heel"}, // 5
                          {"boat_state.euler.pitch", "Pitch"}, // 6
                          {"boat_state.pos.x", "Longitude"}, // 7
                          {"boat_state.pos.y", "Latitude"}, // 8
                          {"boat_state.vel.x", "Vel X"}, // 9
                          {"boat_state.vel.y", "Vel Y"}, // 10
                          {"true_wind.x", "True Wind X"}, // 11
                          {"true_wind.y", "True Wind Y"}, // 12
                          {"can126992.sys_time.time", "True Time"}, // 13
                          {"heading_cmd.heading", "Heading Cmd"}, // 14
                          {"control_mode.rudder_mode", "Rudder Mode"}, // 15
                          }, FLAGS_csv_file, 0.1);
  csv.Stop();
  csv.Start();
  sailbot::Pong pong(&csv);

#endif
  std::thread rthread(&sailbot::LogReplay::Run, &replay);
#if 1
  std::thread pthread(&sailbot::Pong::Run, &pong);
  std::thread csvthread(&sailbot::CsvLogger::Run, &csv);
#endif

  server.Run();

  rthread.join();
#if 1
  pthread.join();
  csvthread.join();
#endif
}
