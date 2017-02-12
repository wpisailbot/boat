#pragma once

#include "gtest/gtest.h"
#include "ipc/queue.hpp"
#include "util/clock.h"

namespace sailbot {
namespace testing {

class TestWrapper : public ::testing::Test {
 protected:
  TestWrapper();
  ~TestWrapper();

  void Sleep(double sec) {
    clock_->SleepUntil(clock_->Time() +
                       std::chrono::milliseconds(int(sec * 1e3)));
  }

  void TearDown() override {
    util::RaiseShutdown();
  }

 private:
  std::unique_ptr<util::ClockInstance> clock_;

  std::unique_ptr<std::thread> clock_manager_thread_;
};

}  // namespace testing
}  // namespace sailbot
