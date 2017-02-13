#include "clock.h"

#include <stdio.h>
#include <signal.h>
#include <sys/resource.h>
#include <sys/time.h>

#include "glog/logging.h"
#include "gflags.h"

namespace sailbot {
namespace util {

bool monotonic_clock::fake_clock = false;
std::atomic<monotonic_clock::rep> monotonic_clock::time_;
std::condition_variable_any monotonic_clock::tick_;
monotonic_clock::rep monotonic_clock::next_wakeup_ = 0;
std::mutex monotonic_clock::wakeup_time_mutex_;

std::shared_timed_mutex ClockInstance::m_;

Loop::Loop(float period)
    : period_(monotonic_clock::rep(period * 1e9)),
      last_trigger_(clock_.Time()) {}

void Loop::WaitForNext() {
  monotonic_clock::time_point time;
  do {
    // TODO(james): Avoid potential infinite loop.
    time = clock_.Time();
    last_trigger_ += period_;
  } while (time > last_trigger_);
  clock_.SleepUntil(last_trigger_);
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
    set_wakeup(len);
    tick_.notify_all();
    tick_.wait(l, [&]() {
      set_wakeup(len);
      return len <= time_ || IsShutdown();
    });
  }
}

ClockInstance::ClockInstance() : lck_(m_) {}

monotonic_clock::time_point ClockInstance::Time() {
  return monotonic_clock::now();
}

void ClockInstance::SleepUntil(monotonic_clock::time_point time) {
  monotonic_clock::sleep_until(time, lck_);
}

void ClockManager::Run(monotonic_clock::rep start_time) {
  bool first_run = true;
  monotonic_clock::set_time(start_time);
  if (monotonic_clock::is_fake()) {
    std::unique_lock<std::shared_timed_mutex> lck(ClockInstance::m_);
    while (!IsShutdown()) {
      if (!first_run) {
        // Allow the clockmanager to shut down.
        monotonic_clock::tick_.wait_for(lck, std::chrono::milliseconds(100));
        if (IsShutdown())
          break;
      }
      first_run = false;
      monotonic_clock::set_time(monotonic_clock::next_wakeup_/*, lck*/);
    }
    // TODO(james): This probably should be set_time(+Infinity), not next_wakeup.
    monotonic_clock::set_time(monotonic_clock::next_wakeup_/*, lck*/);
  }
}

namespace {
  std::atomic<bool> done{false};
}

void SignalHandler(int signum) {
  RaiseShutdown();
}

void CancelShutdown() {
  done = false;
}

void Init(int argc, char *argv[]) {
  if (signal(SIGINT, &SignalHandler) == SIG_ERR) {
    LOG(FATAL) << "Failed to create signal handler";
  }
  /*
  struct sigaction sig_handler;
  sig_handler.sa_handler = SignalHandler;
  PCHECK(sigaction(SIGINT, &sig_handler, NULL) != -1) << "Failed to create signal handler";
  */
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
}

void SetCurrentThreadRealtimePriority(int priority) {
  // Limit runtime of any single thread
  struct rlimit rlim;
  getrlimit(RLIMIT_RTTIME, &rlim);
  rlim.rlim_cur = 500000; // half a second
  setrlimit(RLIMIT_RTTIME, &rlim);

  struct sched_param param;
  param.sched_priority = priority;
  PCHECK(sched_setscheduler(0, SCHED_FIFO, &param) != -1)
      << "sched_setscheduler failed with SCHED_FIFO and priority = "
      << priority;
}

bool IsShutdown() { return done; }
void RaiseShutdown() {
  done = true;
  util::monotonic_clock::tick_.notify_all();
}

}  // util
}  // sailbot
