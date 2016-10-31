// Author: James Kuszmaul <jabukuszmaul@gmail.com>
#pragma once

#include <time.h>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <set>

namespace sailbot {
namespace util {

class monotonic_clock {
 public:
  typedef std::chrono::nanoseconds duration;
  typedef duration::rep rep;
  typedef duration::period period;
  typedef std::chrono::time_point<monotonic_clock> time_point;

  static constexpr clockid_t clock_ = CLOCK_MONOTONIC;
  static constexpr bool is_steady = true;

  static time_point now();

  static void sleep_until(time_point time,
                          std::shared_lock<std::shared_timed_mutex>& l);

  static bool is_fake() { return fake_clock; }

 private:
  static bool fake_clock;
  static std::atomic<rep> time_;
  static std::condition_variable_any tick_;
  static rep next_wakeup_;
  static std::mutex wakeup_time_mutex_;

  static void set_time(rep time) {
    time_ = time;
    tick_.notify_all();
  }

  friend class ClockManager;
};

class ClockInstance {
 public:
  ClockInstance();
  inline monotonic_clock::time_point Time();
  void SleepUntil(monotonic_clock::time_point time);
 private:
  static std::shared_timed_mutex m_;
  std::shared_lock<std::shared_timed_mutex> lck_;

  friend class ClockManager;
};

class ClockManager {
 public:
  static void SetFakeClock(bool is_fake) {
    monotonic_clock::fake_clock = is_fake;
  }
  static void Run();
};

/**
 * A class that allows you to construct a periodic loop, based on
 * clock_nanosleep. Calling WaitForNext() will sleep until the next
 * period.
 * TODO(james): Note/count missed cycles.
 */
class Loop {
 public:
  Loop(float period);
  void WaitForNext();
 private:
  ClockInstance clock_;
  monotonic_clock::duration period_;
  monotonic_clock::time_point last_trigger_;
};

}  // util
}  // sailbot
