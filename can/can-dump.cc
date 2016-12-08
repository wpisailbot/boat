#include "can.h"
#include <thread>


int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);
  sailbot::can::CanNode can;
  std::thread t(&sailbot::can::CanNode::Run, &can);
  t.join();
}
