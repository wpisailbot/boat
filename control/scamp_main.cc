#include "scamp.h"

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);

  sailbot::SCAMP scamp;
  scamp.Run();
}
