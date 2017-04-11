#include "rigid_wing.h"
#include "gflags.h"

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);
  sailbot::RigidWing wing;
  wing.Run();
}
