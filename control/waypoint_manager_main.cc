#include "waypoint_manager.h"

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);

  sailbot::control::WaypointManager manager;
  manager.Run();
}
