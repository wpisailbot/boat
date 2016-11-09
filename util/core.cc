#include "core.h"
#include <signal.h>

#include "glog/logging.h"

namespace sailbot {

namespace {
  std::atomic<bool> done{false};
  void SignalHandler(int signum) {
    done = true;
  }
}

void Init() {
  if (signal(SIGINT, &SignalHandler) == SIG_ERR) {
    LOG(FATAL) << "Failed to create signal handler";
  }
}

bool IsShutdown() { return done; }
void RaiseShutdown() { done = true; }

}  // namespace sailboat
