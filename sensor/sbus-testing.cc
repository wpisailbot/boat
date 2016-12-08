#include "read-sbus.h"

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);

  sailbot::ReadSBUS sbus("/dev/ttyO4");
  sbus.Run();
}
