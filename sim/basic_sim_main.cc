#include "sim_inter.h"
#include "csv_logging.h"
#include <gflags/gflags.h>
#include "control/simple.h"

DEFINE_string(csv_file, "sim/python/basic_sim_data.csv", "File to save CSV data to");
/**
 * A simple simulation that runs with no change to the inputs, just to see what
 * happens when nothing is being done.
 */
int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);
  sailbot::util::ClockManager::SetFakeClock(true);
  sailbot::sim::SimulatorNode sim;
  sailbot::control::SimpleControl ctrl;
  sim.set_wind(-M_PI / 2, 6);
#define LOG_VECTOR(path, name)                                                 \
  { path ".x", name " X" }                                                     \
  , {path ".y", name " Y"}, {                                                  \
  path ".z", name " Z"}
  sailbot::CsvLogger csv({
                          {"boat_state.internal.sail", "Sail"},
                          {"boat_state.internal.rudder", "Rudder"},
                          {"boat_state.pos.x", "Boat X"},
                          {"boat_state.pos.y", "Boat Y"},
                          {"boat_state.vel.x", "Boat Vel X"},
                          {"boat_state.vel.y", "Boat Vel Y"},
                          {"boat_state.orientation.w", "Quat W"},
                          {"boat_state.orientation.x", "Quat X"},
                          {"boat_state.orientation.y", "Quat Y"},
                          {"boat_state.orientation.z", "Quat Z"},
                          {"wind.x", "Wind X"},
                          {"wind.y", "Wind Y"},
                          LOG_VECTOR("sim_debug.fs", "Sail Force"),
                          LOG_VECTOR("sim_debug.fr", "Rudder Force"),
                          LOG_VECTOR("sim_debug.fk", "Keel Force"),
                          LOG_VECTOR("sim_debug.fh", "Hull Force"),
                          LOG_VECTOR("sim_debug.fnet", "Net Force"),
                          LOG_VECTOR("sim_debug.taus", "Sail Torque"),
                          LOG_VECTOR("sim_debug.taur", "Rudder Torque"),
                          LOG_VECTOR("sim_debug.tauk", "Keel Torque"),
                          LOG_VECTOR("sim_debug.tauh", "Hull Torque"),
                          LOG_VECTOR("sim_debug.tauright", "Righting Moment"),
                          LOG_VECTOR("sim_debug.taunet", "Net Torque"),
                         },
                         FLAGS_csv_file, .01);
#undef LOG_VECTOR

  std::thread clock(&sailbot::util::ClockManager::Run);
  clock.detach();

  std::thread t(&sailbot::CsvLogger::Run, &csv);
  std::thread t_ctrl(&sailbot::control::SimpleControl::Run, &ctrl);
  sim.Run();

  t_ctrl.join();
  t.join();
}
