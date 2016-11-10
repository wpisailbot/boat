#include "sim_inter.h"
#include "csv_logging.h"

/**
 * A simple simulation that runs with no change to the inputs, just to see what
 * happens when nothing is being done.
 */
int main(int argc, char *argv[]) {
  const char *file = "sim/python/basic_sim_data.csv";
  if (argc > 1) {
    file = argv[1];
  }
  sailbot::util::Init();
  sailbot::util::ClockManager::SetFakeClock(true);
  sailbot::sim::SimulatorNode sim;
  sim.set_wind(-1.5, 3);
  sailbot::CsvLogger csv({{"boat_state.internal.sail", "Sail"},
                          {"boat_state.internal.rudder", "Rudder"},
                          {"boat_state.pos.x", "Boat X"},
                          {"boat_state.pos.y", "Boat Y"},
                          {"boat_state.orientation.w", "Quat W"},
                          {"boat_state.orientation.x", "Quat X"},
                          {"boat_state.orientation.y", "Quat Y"},
                          {"boat_state.orientation.z", "Quat Z"}},
                         file);

  std::thread clock(&sailbot::util::ClockManager::Run);
  clock.detach();

  std::thread t(&sailbot::CsvLogger::Run, &csv);
  sim.Run();

  t.join();
}
