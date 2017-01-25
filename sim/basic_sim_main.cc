#include "sim_inter.h"
#include "csv_logging.h"
#include <gflags/gflags.h>
#include "control/simple.h"
#include "control/line_tacking.h"

DEFINE_string(csv_file, "sim/python/basic_sim_data.csv", "File to save CSV data to");
/**
 * A simple simulation that runs with no change to the inputs, just to see what
 * happens when nothing is being done.
 */
int main(int argc, char *argv[]) {
  sailbot::Queue::set_testing(true);
  sailbot::util::Init(argc, argv);
  sailbot::util::ClockManager::SetFakeClock(true);
  sailbot::sim::SimulatorNode sim;
  sailbot::control::SimpleControl ctrl(true);
  sailbot::control::LineTacker tacker;
  sim.set_wind(-M_PI / 2, 4);
#define LOG_VECTOR(path, name)                                                 \
  { path ".x", name " X" }                                                     \
  , {path ".y", name " Y"}, {                                                  \
  path ".z", name " Z"}
  sailbot::CsvLogger csv({
                          {"boat_state.internal.sail", "Sail"}, // 0
                          {"boat_state.internal.rudder", "Rudder"}, // 1
                          {"boat_state.pos.x", "Boat X"}, // 2
                          {"boat_state.pos.y", "Boat Y"}, // 3
                          {"boat_state.vel.x", "Boat Vel X"}, // 4
                          {"boat_state.vel.y", "Boat Vel Y"}, // 5
                          {"boat_state.orientation.w", "Quat W"}, // 6
                          {"boat_state.orientation.x", "Quat X"}, // 7
                          {"boat_state.orientation.y", "Quat Y"}, // 8
                          {"boat_state.orientation.z", "Quat Z"}, // 9
                          {"wind.x", "Wind X"}, // 10
                          {"wind.y", "Wind Y"}, // 11
                          LOG_VECTOR("sim_debug.fs", "Sail Force"), // 12-14
                          LOG_VECTOR("sim_debug.fr", "Rudder Force"), // 15-17
                          LOG_VECTOR("sim_debug.fk", "Keel Force"), // 18-20
                          LOG_VECTOR("sim_debug.fh", "Hull Force"), // 21-23
                          LOG_VECTOR("sim_debug.fnet", "Net Force"), // 24-26
                          LOG_VECTOR("sim_debug.taus", "Sail Torque"), // 27-29
                          LOG_VECTOR("sim_debug.taur", "Rudder Torque"), // 30-32
                          LOG_VECTOR("sim_debug.tauk", "Keel Torque"), // 33-35
                          LOG_VECTOR("sim_debug.tauh", "Hull Torque"), // 36-38
                          LOG_VECTOR("sim_debug.tauright", "Righting Moment"), // 39-41
                          LOG_VECTOR("sim_debug.taunet", "Net Torque"), // 42-44
                          {"heading_cmd.heading", "Heading Goal"}, // 45
                         },
                         FLAGS_csv_file, .01);
#undef LOG_VECTOR

  std::thread clock(&sailbot::util::ClockManager::Run, 0);
  clock.detach();

  std::thread t(&sailbot::CsvLogger::Run, &csv);
  std::thread t_ctrl(&sailbot::control::SimpleControl::Run, &ctrl);
  std::thread t_tack(&sailbot::control::LineTacker::Run, &tacker);
  sim.Run();

  t_ctrl.join();
  t_tack.join();
  t.join();
}
