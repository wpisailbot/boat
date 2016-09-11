#include "gtest/gtest.h"
#include "clock.h"
#include <chrono>
#include <stdio.h>

TEST(TimeTest, BasicLooping) {
  ::std::chrono::time_point<::std::chrono::steady_clock> start, cur;

  double period = .001;
  Loop loop({0, period * 1e9});

  start = ::std::chrono::steady_clock::now();
  double last_elapsed = 0;
  for (int i = 0; i < 100; ++i) {
    loop.WaitForNext();

    cur = ::std::chrono::steady_clock::now();
    ::std::chrono::duration<double> elapsed = cur - start;
    double difftime = elapsed.count();
    EXPECT_NEAR(difftime, period, .0002);
    start = cur;
  }
}
