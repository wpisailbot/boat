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
  sailbot::sim::SimulatorNode sim(-1);
  sailbot::control::SimpleControl ctrl(true);
  sailbot::control::LineTacker tacker;
  sim.set_wind(-M_PI / 2, 4);
#define LOG_VECTOR(path, name)                                                 \
  { path ".x", name " X" }                                                     \
  , {path ".y", name " Y"}, {                                                  \
  path ".z", name " Z"}
#define TRUE_STATE "sim_true_boat_state"
  sailbot::CsvLogger csv({
                          {TRUE_STATE".internal.sail", "Sail"}, // 0
                          {TRUE_STATE".internal.rudder", "Rudder"}, // 1
                          {TRUE_STATE".pos.x", "Boat X"}, // 2
                          {TRUE_STATE".pos.y", "Boat Y"}, // 3
                          {TRUE_STATE".vel.x", "Boat Vel X"}, // 4
                          {TRUE_STATE".vel.y", "Boat Vel Y"}, // 5
                          {TRUE_STATE".orientation.w", "Quat W"}, // 6
                          {TRUE_STATE".orientation.x", "Quat X"}, // 7
                          {TRUE_STATE".orientation.y", "Quat Y"}, // 8
                          {TRUE_STATE".orientation.z", "Quat Z"}, // 9
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
#undef TRUE_STATE
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
