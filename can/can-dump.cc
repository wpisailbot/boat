#include "can.h"
#include <thread>


int main(int argc, char *argv[]) {
  sailbot::util::SetCurrentThreadRealtimePriority(10);
  sailbot::util::Init(argc, argv);
  sailbot::can::CanNode can;
  can.Run();
}
