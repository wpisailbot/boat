#include "sim_inter.h"
#include "csv_logging.h"

/**
 * A simple simulation that runs with no change to the inputs, just to see what
 * happens when nothing is being done.
 */
int main(int argc, char *argv[]) {
  sailbot::Init();
  sailbot::util::ClockManager::SetFakeClock(true);
  sailbot::sim::SimulatorNode sim;
  sim.set_wind(0., 3);
  sailbot::CsvLogger csv(
      {{"boat_state.internal.sail", "Sail"}, {"boat_state.pos.x", "Boat X"}}, "test_file.csv");

  std::thread clock(&sailbot::util::ClockManager::Run);
  clock.detach();

  std::thread t(&sailbot::CsvLogger::Run, &csv);
  sim.Run();

  t.join();
}
