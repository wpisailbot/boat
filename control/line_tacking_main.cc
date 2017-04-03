#include "line_tacking.h"

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);
  sailbot::control::LineTacker tacker;
  tacker.Run();
}
