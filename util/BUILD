cc_library(
  name="clock",
  srcs=["clock.cc"],
  hdrs=["clock.h"],
)
cc_test(
  name="clock_test",
  srcs=["clock_test.cc"],
  copts = ["-Iexternal/gtest/include"],
  deps = [
    "@gtest//:main",
  ],
)
