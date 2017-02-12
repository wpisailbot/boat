#include "gtest/gtest.h"
#include "clock.h"
#include <chrono>
#include <stdio.h>

namespace sailbot {
namespace util {
namespace testing {

TEST(TimeTest, BasicLooping) {
  ::std::chrono::time_point<::std::chrono::steady_clock> start, cur;

  double period = .01;
  Loop loop(period);

  start = ::std::chrono::steady_clock::now();
  double last_elapsed = 0;
  for (int i = 0; i < 100; ++i) {
    loop.WaitForNext();

    cur = ::std::chrono::steady_clock::now();
    ::std::chrono::duration<double> elapsed = cur - start;
    double difftime = elapsed.count();
    EXPECT_NEAR(difftime, period, .001);
    start = cur;
  }
}

}  // testing
}  // util
}  // sailbot
