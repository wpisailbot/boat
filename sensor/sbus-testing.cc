#include "read-sbus.h"
#include "gflags.h"

DEFINE_string(sbus_port, "/dev/ttyO4", "Serial port for SBUS input");

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);

  sailbot::ReadSBUS sbus(FLAGS_sbus_port.c_str());
  sbus.Run();
}
