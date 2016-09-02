#include "clock.h"

namespace {
  bool time_is_greater(timespec rhs, timespec lhs) {
    return rhs.tv_sec == lhs.tv_sec ? rhs.tv_nsec > lhs.tv_nsec
                                    : rhs.tv_sec > lhs.tv_sec;
  }
}  // namespace

Loop::Loop(struct timespec period, clockid_t clockid)
    : clock_(clockid), period_(period) {
  if (clock_gettime(clock_, &last_trigger_)) {
    LOG(FATAL, "Failed to fetch clock time.");
  }
}

bool Loop::WaitForNext() {
  timespec time;
  do {
    clock_gettime(clock_, &last_trigger_);
  } while (time_is_greater(time, last_trigger_));
  clock_nanosleep(clock_,
}
