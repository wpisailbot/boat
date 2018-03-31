#include "monitor.h"

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);
  sailbot::Monitor monitor;
  monitor.Run();
}
