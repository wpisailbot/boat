#include "simple.h"

int main(int argc, char *argv[]) {
  sailbot::util::SetCurrentThreadRealtimePriority(10);
  sailbot::util::Init(argc, argv);

  sailbot::control::SimpleControl simple(true/*do_rudder*/);
  simple.Run();
}
