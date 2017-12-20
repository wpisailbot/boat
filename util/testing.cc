#include "testing.h"

namespace sailbot {
namespace testing {

TestWrapper::TestWrapper() {
  util::CancelShutdown();
  Queue::set_testing(true);
  sailbot::util::ClockManager::SetFakeClock(true, true);

  clock_.reset(new util::ClockInstance());

  clock_manager_thread_.reset(new std::thread(&sailbot::util::ClockManager::Run, 0));
}

TestWrapper::~TestWrapper() {
  clock_manager_thread_->join();
}

}  // namespace testing
}  // namespace sailbot
