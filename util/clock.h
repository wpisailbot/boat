// Author: James Kuszmaul <jabukuszmaul@gmail.com>
#pragma once

#include <time.h>
#include <chrono>
#include <thread>
#include <queue>
#include <condition_variable>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <set>

namespace sailbot {
namespace util {

/**
 * All of these classes have to do with timing and sleeping in the robot. In the
 * nominal case where the code is running in real life, most of this is a
 * relatively light wrapper over underlying calls to clock_nanosleep and
 * clock_gettime with the right flags.
 * This also contains the functions for the signal handling and global
 * initialization of the process state.
 *
 * The more complex portion of this entire file is in handling the spoofing of
 * system time for purposes of simulation and testing.
 *
 * General structure of the system time spoofing:
 *
 * monotonic_clock maintains the current time. This time is updated on the
 * assumption that every thread actively running either doesn't care about time
 * or has calls to monotonic_clock::sleep_until() occurring.
 * All the different threads should, when they call sleep_until() be supplying
 * the same, already locked as reader, shared mutex (this is automated by
 * ClockInstance, which uses the static ClockInstance::m_). At all times,
 * we maintain a next_wakeup, which is the earliest "time" that a thread
 * will need to be awaken. Inside sleep_until, we pass on the lock to
 * a condition variable which waits on the mutex.
 * Once every thread that is maintaining a lock on ClockInstance::m_ has called
 * sleep_until (or otherwise released their lock), ClockManager::Run (which must
 * have been started by the process at some point) will obtain a writer's lock
 * on the shared mutex and update the internal monotonic_clock time to
 * monotonic_clock::next_wakeup_ and send out a notify_all on the condition
 * variables.
 * This will cause all the sleep_until's to wake up and check to see if their
 * time has arrived. If it has, then they return and carry on; otherwise, they
 * update next_wakeup_ (if necessary) and go back to waiting.
 *
 * For examples on usage, see util/testing.*
 *
 */

class monotonic_clock {
 public:
  typedef std::chrono::nanoseconds duration;
  // Our rep corresponds to one nanosecond.
  typedef duration::rep rep;
  typedef duration::period period;
  typedef std::chrono::time_point<monotonic_clock> time_point;

  static constexpr clockid_t clock_ = CLOCK_MONOTONIC;
  static constexpr bool is_steady = true;

  static time_point now();

  /*
   * Sleep until a particular time, passing in an already locked shared lock l
   * (which is only relevant when using a fake clock).
   */
  static void sleep_until(time_point time,
                          std::shared_lock<std::shared_timed_mutex>& l);

  static bool is_fake() { return fake_clock; }

  /*
   * If running a fake clock, the time at which the nex tsleeping thread needs
   * to be woken up.
   */
  static rep next_wakeup() {
    std::unique_lock<std::mutex> lck(wakeup_time_mutex_);
    return wakeup_times_.empty() ? time_.load() : wakeup_times_.top();
  }

 private:
  static bool fake_clock;

  // All of these member variables and methods are for when running the fake
  // clock:

  // Current time
  static std::atomic<rep> time_;
  // condition variable to wake threads up
  static std::condition_variable_any tick_;
  static rep next_wakeup_;
  static std::mutex wakeup_time_mutex_;
  static std::priority_queue<rep, std::vector<rep>, std::greater<rep>>
      wakeup_times_;

  // ClockInstance::m_ MUST be locked with a unique_lock if attempting to call
  // set_time. This is to protect the interplay between time_ and next_wakeup_.
  static void set_time(rep time/*,
                       std::unique_lock<std::shared_timed_mutex>& unlock_lock*/) {
    time_ = time;
    tick_.notify_all();
  }

  // ClockInstance::m_ MUST be locked as a reader if calling set_wakeup, to
  // avoid potential issues with changing time_ while next_wakeup_ is being
  // changed
  static void set_wakeup(rep time) {
    std::unique_lock<std::mutex> lck(wakeup_time_mutex_);
    if (time > time_) {
      wakeup_times_.push(time);
    }
    while (!wakeup_times_.empty() && wakeup_times_.top() < time_) {
      wakeup_times_.pop();
    }
  }

  friend class ClockManager;
  friend void RaiseShutdown();
};

/**
 * A clock for sleeping and getting current time.
 * ONLY USE ONE PER THREAD; if you are running in a Node,
 * then you already have access to a Time() function and
 * you shouldn't be sleeping yourself.
 * If you use more than one of these per thread, then
 * you can never have two ClockInstance's sleeping at
 * once, and as a consequence the fake clock will fail to work
 * properly.
 * TODO(james): Allow more than one ClockInstance per thread
 */
class ClockInstance {
 public:
  ClockInstance();
  ~ClockInstance();
  monotonic_clock::time_point Time();
  void SleepUntil(monotonic_clock::time_point time);
  /**
   * Performs all cleanup associated with this clock.
   * In particular, unlocks the shared lock on m_,
   * so that the fake clock can continue being iterated without
   * expecting this instance to do anything.
   */
  void CleanUp();

 private:
  // The shared mutex so that we know when everyone has gone to sleep and we
  // can update the fake clock.
  static std::shared_timed_mutex m_;
  // Our individual lock on the mutex; locked from construction until
  // CleanUp(), except when sleeping.
  std::shared_lock<std::shared_timed_mutex> lck_;

  friend class ClockManager;
};

/**
 * Takes care of running the fake clock (well, and the real one, but nothing
 * really needs to be *done* for the real clock).
 */
class ClockManager {
 public:
  static void SetFakeClock(bool is_fake, bool set_start_time=false) {
    monotonic_clock::fake_clock = is_fake;
    if (set_start_time) {
      monotonic_clock::set_time(0);
    }
  }
  /**
   * Manage the fake clock; start up in a thread of its own.
   */
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
  /**
   * Construct a loop that loops with a period of period seconds.
   */
  Loop(float period);
  /*
   * Wait for the next loop iteration to occur.
   */
  void WaitForNext();
  /**
   * Perform cleanup when we are done looping. Mostly important with
   * the fake clock.
   */
  void Done() { clock_.CleanUp(); }
  monotonic_clock::time_point Time() { return clock_.Time(); }

 private:
  ClockInstance clock_;
  monotonic_clock::duration period_;
  monotonic_clock::time_point last_trigger_;
};

// Should be called at the start of every PROCESS (not Node).
void Init(int argc, char *argv[]);
// Returns whether the process is terminating and the current thread should
// attempt to exit.
bool IsShutdown();
// For testing purposes, resets the flag states so that we are no longer
// shutting down.
// TODO(james): This is poor style, as it relies on us remembering to call
//   it in testing to clear static variable state. Instead, require some
//   form of initialization on every test to statically enforce this.
void CancelShutdown();
// Indicate to other threads that it is time to shut down.
void RaiseShutdown();
// Sets thread priority.
void SetCurrentThreadRealtimePriority(int priority);

}  // util
}  // sailbot
