#include "line_plan.h"

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);

  sailbot::control::LinePlan planner;
  planner.Run();
}
