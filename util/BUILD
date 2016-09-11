cc_library(
  name="clock",
  srcs=["clock.cc"],
  hdrs=["clock.h"],
  linkopts = ["-lglog"],
)
cc_test(
  name="clock_test",
  srcs=["clock_test.cc"],
  copts = ["-Iexternal/gtest/include"],
  deps = [
    "@gtest//:main",
    ":clock",
  ],
  flaky = True,
)
