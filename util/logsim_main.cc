#include "log-replay.h"
#include "ui/server.h"
#include "sim/csv_logging.h"
#include "sim/sim_inter.h"
#include "sensor/state_estimator.h"
#include <iomanip>
#include <gflags/gflags.h>

DEFINE_string(csv_file, "/tmp/basic_replay_data.csv", "File to save CSV data to");
DEFINE_double(speedup, 1000, "Factor by which to speed up the replay");

namespace sailbot {
class Pong : public Node {
 public:
  Pong() : Node(0.1), sail_queue_("sail_cmd", true) {
    RegisterHandler<msg::BoatState>("orig_boat_state",
                                    [this](const msg::BoatState &msg) {
      msg::SailCmd cmd;
      cmd.set_pos(msg.internal().sail());
      sail_queue_.send(&cmd);
    });
  }

 private:
  virtual void Iterate() {
    usleep(1e5 / FLAGS_speedup);
  }

  ProtoQueue<msg::SailCmd> sail_queue_;
};
}  // sailbot

int main(int argc, char *argv[]) {
  sailbot::Queue::set_testing(true);
  sailbot::util::Init(argc, argv);

  // Construct everything else *after* replay is constructed, to avoid issues
  // with clock timing.
  sailbot::LogReplay replay({
                             {"true_wind", "sim_set_wind"},
                             {"boat_state", "orig_boat_state"},
                             {"heading_cmd", "heading_cmd"},
                             {"rudder_cmd", "rudder_cmd"},
                            },
                            true);
  sailbot::sim::SimulatorNode sim(5);
  sailbot::WebSocketServer server;
  sailbot::control::StateEstimator state_estimator;
  sailbot::CsvLogger csv(
      {
       {"wind.x", "Wind X"},                               // 0
       {"wind.y", "Wind Y"},                               // 1
       {"sim_true_boat_state.internal.sail", "Sail"},      // 2
       {"sim_true_boat_state.internal.rudder", "Rudder"},  // 3
       {"sim_true_boat_state.euler.yaw", "Yaw"},           // 4
       {"sim_true_boat_state.euler.roll", "Heel"},         // 5
       {"sim_true_boat_state.euler.pitch", "Pitch"},       // 6
       {"sim_true_boat_state.pos.x", "Longitude"},         // 7
       {"sim_true_boat_state.pos.y", "Latitude"},          // 8
       {"sim_true_boat_state.vel.x", "Vel X"},             // 9
       {"sim_true_boat_state.vel.y", "Vel Y"},             // 10
       {"true_wind.x", "True Wind X"},                     // 11
       {"true_wind.y", "True Wind Y"},                     // 12
       {"can126992.sys_time.time", "True Time"},           // 13
       {"heading_cmd.heading", "Heading Cmd"},             // 14
       {"control_mode.rudder_mode", "Rudder Mode"},        // 15
       {"orig_heading_cmd.heading", "Orig Heading Cmd"},   // 16
       {"orig_boat_state.internal.sail", "Orig Sail"},     // 17
       {"orig_boat_state.internal.rudder", "Orig Rudder"}, // 18
       {"orig_boat_state.euler.yaw", "Orig Yaw"},          // 19
       {"orig_boat_state.euler.roll", "Orig Heel"},        // 20
       {"orig_boat_state.euler.pitch", "Orig Pitch"},      // 21
       {"orig_boat_state.pos.x", "Orig Longitude"},        // 22
       {"orig_boat_state.pos.y", "Orig Latitude"},         // 23
       {"orig_boat_state.vel.x", "Orig Vel X"},            // 24
       {"orig_boat_state.vel.y", "Orig Vel Y"},            // 25
      },
      FLAGS_csv_file, 0.1);
  sailbot::Pong pong;

  std::thread rthread(&sailbot::LogReplay::Run, &replay);
  std::thread pthread(&sailbot::Pong::Run, &pong);
  std::thread csvthread(&sailbot::CsvLogger::Run, &csv);
  std::thread simthread(&sailbot::sim::SimulatorNode::Run, &sim);
  std::thread statethread(&sailbot::control::StateEstimator::Run, &state_estimator);

  server.Run();

  rthread.join();
  pthread.join();
  csvthread.join();
  simthread.join();
  statethread.join();
}
