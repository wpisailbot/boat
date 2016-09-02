#pragma once

#include <time.h>

class Loop {
 public:
  Loop(struct timespec period, clockid_t clockid=CLOCK_MONOTONIC);
  bool WaitForNext();
 private:
  clockid_t clock_;
  struct timespec period_;
  struct timespec last_trigger_;
};
