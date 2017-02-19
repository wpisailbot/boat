#include "rudder.h"

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);
  sailbot::control::RudderController r;
  r.Run();
}
