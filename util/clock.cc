#include "clock.h"

#include <stdio.h>

#include "glog/logging.h"

namespace sailbot {
namespace util {

bool monotonic_clock::fake_clock = false;

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

monotonic_clock::time_point monotonic_clock::now() {
  if (!fake_clock) {
    timespec time;
    clock_gettime(clock_, &time);
    return time_point(duration(time.tv_nsec + time.tv_sec * 1000000000LL));
  } else {
    return time_point(duration(time_));
  }
}

void monotonic_clock::sleep_until(time_point time,
                                  std::shared_lock<std::shared_timed_mutex> &l) {
  rep len = time.time_since_epoch().count();
  if (!fake_clock) {
    timespec ts;
    ts.tv_sec = len / 1000000000LL;
    ts.tv_nsec = len - ts.tv_sec * 1000000000LL;
    clock_nanosleep(clock_, TIMER_ABSTIME /*flags*/, &ts, NULL);
  } else {
    {
      std::unique_lock<std::mutex> lck(wakeup_time_mutex_);
      if (next_wakeup_ < time_) {
        next_wakeup_ = std::max(len, time_.load());
      } else {
        next_wakeup_ = std::min(next_wakeup_, len);
      }
    }
    tick_.wait(l, [&]() { return len <= time_; });
  }
}

ClockInstance::ClockInstance() : lck_(m_) {}

monotonic_clock::time_point ClockInstance::Time() {
  return monotonic_clock::now();
}

void ClockInstance::SleepUntil(monotonic_clock::time_point time) {
  monotonic_clock::sleep_until(time, lck_);
}

void ClockManager::Run() {
  if (monotonic_clock::is_fake()) {
    while (true) {
      std::unique_lock<std::shared_timed_mutex> lck(ClockInstance::m_);
      monotonic_clock::set_time(monotonic_clock::next_wakeup_);
    }
  }
}

}  // util
}  // sailbot
