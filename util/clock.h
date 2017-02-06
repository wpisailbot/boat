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
  static rep next_wakeup() { return next_wakeup_; }

 private:
  static bool fake_clock;
  static std::atomic<rep> time_;
  static std::condition_variable_any tick_;
  static rep next_wakeup_;
  static std::mutex wakeup_time_mutex_;

  static void set_time(rep time/*,
                       std::unique_lock<std::shared_timed_mutex>& unlock_lock*/) {
    time_ = time;
    // TODO(james): Locking structure still isn't correct.
    // TODO(james): Figure out if the above comment still applies.
    //unlock_lock.unlock();
    tick_.notify_all();
  }

  friend class ClockManager;
  friend void RaiseShutdown();
};

class ClockInstance {
 public:
  ClockInstance();
  ~ClockInstance() {
    CleanUp();
  }
  monotonic_clock::time_point Time();
  void SleepUntil(monotonic_clock::time_point time);
  void CleanUp() {
    if (lck_) {
      lck_.unlock();
    }
  }

 private:
  static std::shared_timed_mutex m_;
  std::shared_lock<std::shared_timed_mutex> lck_;

  friend class ClockManager;
};

class ClockManager {
 public:
  static void SetFakeClock(bool is_fake, bool set_start_time=false) {
    monotonic_clock::fake_clock = is_fake;
    if (set_start_time) {
      monotonic_clock::set_time(0);
    }
  }
  static void Run(monotonic_clock::rep start_time);
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
  void Done() { clock_.CleanUp(); }

 private:
  ClockInstance clock_;
  monotonic_clock::duration period_;
  monotonic_clock::time_point last_trigger_;
};

// Should be called at the start of every PROCESS (not Node).
void Init(int argc, char *argv[]);
bool IsShutdown();
void CancelShutdown();
void RaiseShutdown();
void SetCurrentThreadRealtimePriority(int priority);

}  // util
}  // sailbot
