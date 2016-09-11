#include "clock.h"

#include <stdio.h>

#include "glog/logging.h"

namespace {
  bool time_is_greater(timespec rhs, timespec lhs) {
    return rhs.tv_sec == lhs.tv_sec ? rhs.tv_nsec > lhs.tv_nsec
                                    : rhs.tv_sec > lhs.tv_sec;
  }
  void add_time(timespec *a, timespec *diff) {
    // Assumes valid timespecs (ie, can't be negative).
    a->tv_nsec += diff->tv_nsec;
    a->tv_sec += diff->tv_sec + a->tv_nsec / 1000000000UL;
    a->tv_nsec %= 1000000000UL;
  }
}  // namespace

Loop::Loop(timespec period, clockid_t clockid)
    : clock_(clockid), period_(period) {
  if (clock_gettime(clock_, &last_trigger_)) {
    LOG(FATAL) << "Failed to fetch clock time.";
  }
}

void Loop::WaitForNext() {
  timespec time;
  do {
    clock_gettime(clock_, &time);
    add_time(&last_trigger_, &period_);
  } while (time_is_greater(time, last_trigger_));
  clock_nanosleep(clock_, TIMER_ABSTIME /*flags*/, &last_trigger_,
                  NULL /*remain*/);
}
