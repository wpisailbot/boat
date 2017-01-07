#include "logger.h"

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);
  sailbot::Logger log;
  log.Run();
}
