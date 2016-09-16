// Author: James Kuszmaul <jabukuszmaul@gmail.com>
#pragma once

#include <time.h>

namespace sailbot {
namespace util {

class Loop {
 public:
  Loop(timespec period, clockid_t clockid=CLOCK_MONOTONIC);
  void WaitForNext();
 private:
  clockid_t clock_;
  timespec period_;
  timespec last_trigger_;
};

}  // util
}  // sailbot
